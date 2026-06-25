// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_robot_option_node.hpp"

#include <cmath>
#include <cstring>

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::RobotMotion;
using wmx3Api::Io;
using wmx_robot_option::RAD2DEG;
using MotionState = wmx3Api::kinematics::constants::MotionState;
using MotionErrorCode = wmx3Api::kinematics::constants::MotionErrorCode;

WmxRobotOptionNode::WmxRobotOptionNode()
: Node("wmx_robot_option_node")
{
  declareParameters();

  init_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = init_cb_group_;

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&WmxRobotOptionNode::onEngineReady, this, _1), sub_opts);

  robotReadyPub_ = this->create_publisher<std_msgs::msg::Bool>(
    "wmx/robot/ready", ready_qos);

  // Services are created up front so they are discoverable before the engine is
  // ready; each callback guards on initialized_.
  setServoService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/robot/set_servo", std::bind(&WmxRobotOptionNode::setServo, this, _1, _2));
  setSpeedService_ = this->create_service<wmx_ros2_message::srv::SetRobotScalar>(
    "wmx/robot/set_speed", std::bind(&WmxRobotOptionNode::setSpeed, this, _1, _2));
  clearErrorsService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/robot/clear_errors", std::bind(&WmxRobotOptionNode::clearErrors, this, _1, _2));
  stopMotionService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/robot/stop_motion", std::bind(&WmxRobotOptionNode::stopMotion, this, _1, _2));
  exportParamsService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/robot/export_params", std::bind(&WmxRobotOptionNode::exportParams, this, _1, _2));

  jogPoseService_ = this->create_service<wmx_ros2_message::srv::RobotJogPose>(
    "wmx/robot/jog_pose", std::bind(&WmxRobotOptionNode::jogPose, this, _1, _2));
  jogPoseAbsoluteService_ = this->create_service<wmx_ros2_message::srv::RobotJogPose>(
    "wmx/robot/jog_pose_absolute", std::bind(&WmxRobotOptionNode::jogPoseAbsolute, this, _1, _2));
  setPosePtpService_ = this->create_service<wmx_ros2_message::srv::RobotMovePose>(
    "wmx/robot/set_pose_ptp", std::bind(&WmxRobotOptionNode::setPosePtp, this, _1, _2));

  jogAngleService_ = this->create_service<wmx_ros2_message::srv::RobotMoveAngle>(
    "wmx/robot/jog_angle", std::bind(&WmxRobotOptionNode::jogAngle, this, _1, _2));
  jogAngleAbsoluteService_ = this->create_service<wmx_ros2_message::srv::RobotMoveAngle>(
    "wmx/robot/jog_angle_absolute", std::bind(&WmxRobotOptionNode::jogAngleAbsolute, this, _1, _2));
  setAnglePtpService_ = this->create_service<wmx_ros2_message::srv::RobotMoveAngle>(
    "wmx/robot/set_angle_ptp", std::bind(&WmxRobotOptionNode::setAnglePtp, this, _1, _2));

  checkPoseService_ = this->create_service<wmx_ros2_message::srv::RobotCheckPose>(
    "wmx/robot/check_pose", std::bind(&WmxRobotOptionNode::checkPose, this, _1, _2));
  checkAngleService_ = this->create_service<wmx_ros2_message::srv::RobotCheckAngle>(
    "wmx/robot/check_angle", std::bind(&WmxRobotOptionNode::checkAngle, this, _1, _2));

  setCollisionEnableService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/robot/set_collision_enable",
    std::bind(&WmxRobotOptionNode::setCollisionEnable, this, _1, _2));
  setFittingParamService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/robot/set_fitting_param", std::bind(&WmxRobotOptionNode::setFittingParam, this, _1, _2));
  setCollisionSensitivityService_ = this->create_service<wmx_ros2_message::srv::SetRobotScalar>(
    "wmx/robot/set_collision_sensitivity",
    std::bind(&WmxRobotOptionNode::setCollisionSensitivity, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node waiting for engine...");
}

WmxRobotOptionNode::~WmxRobotOptionNode()
{
  if (statusTimer_) {
    statusTimer_->cancel();
  }
  if (initialized_) {
    robot_.reset();
    io_.reset();
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    } else {
      RCLCPP_INFO(this->get_logger(), "Device closed");
    }
  }
  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node stopped");
}

void WmxRobotOptionNode::declareParameters()
{
  this->declare_parameter<std::string>("robot_xml_path", "");
  this->declare_parameter<std::string>("wmx_param_file_path", "");
  this->declare_parameter<std::string>("robot_export_xml_path", "");
  this->declare_parameter<std::string>("status_frame", "base_link");
  this->declare_parameter<double>("collision_sensitivity", 6.0);
  this->declare_parameter<int>("status_rate", 50);

  this->get_parameter("robot_xml_path", robot_xml_path_);
  this->get_parameter("wmx_param_file_path", wmx_param_file_path_);
  this->get_parameter("status_frame", status_frame_);
  this->get_parameter("collision_sensitivity", collision_sensitivity_);
  this->get_parameter("status_rate", status_rate_);
  if (status_rate_ <= 0) {
    status_rate_ = 50;
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "robot_xml_path: %s", robot_xml_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "wmx_param_file_path: %s", wmx_param_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "status_rate: %d Hz", status_rate_);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

// ---------------------------------------------------------------------------
// Lifecycle / engine attach
// ---------------------------------------------------------------------------

void WmxRobotOptionNode::onEngineReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engine ready — initializing RobotMotion option...");

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to attach to device (lock busy, will retry on next signal).");
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return;
  }

  wmx3Lib_.SetDeviceName("wmx_robot_option_node");
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

  robot_ = std::make_unique<RobotMotion>(&wmx3Lib_);
  io_ = std::make_unique<Io>(&wmx3Lib_);

  if (uploadParams() != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "Failed to upload robot parameters; option not ready.");
    return;
  }
  if (configureDevice() != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure WMX3 parameters; option not ready.");
    return;
  }

  statusPub_ = this->create_publisher<wmx_ros2_message::msg::RobotArmStatus>(
    "wmx/robot/status", 1);
  statusTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / status_rate_),
    std::bind(&WmxRobotOptionNode::statusStep, this));

  initialized_ = true;

  auto ready_msg = std_msgs::msg::Bool();
  ready_msg.data = true;
  robotReadyPub_->publish(ready_msg);

  engineReadySub_.reset();

  RCLCPP_INFO(this->get_logger(), "wmx_robot_option_node is ready (%d Hz)", status_rate_);
}

// Load robot.xml and push the robot model to the engine (port of upload_params).
int WmxRobotOptionNode::uploadParams()
{
  RCLCPP_INFO(this->get_logger(), "Uploading robot parameters from %s", robot_xml_path_.c_str());

  err_ = robot_->mRobotConfig.ImportParamXML(
    const_cast<char *>(robot_xml_path_.c_str()), robotParam_);
  if (err_ != ErrorCode::None) {logErr(err_, "Failed to load robot xml"); return err_;}

  err_ = robot_->mKinematics.SetRobotParam(robotParam_.robotParam);
  if (err_ != ErrorCode::None) {logErr(err_, "Failed to set robot param to engine"); return err_;}

  for (int j = 0; j < robotParam_.robotParam.numJoints; ++j) {
    err_ = robot_->mKinematics.SetFittingData(
      robotParam_.robotParam.robotId, j, robotParam_.fittingData[j]);
    if (err_ != ErrorCode::None) {
      logErr(err_, "Failed to set fitting data of joint " + std::to_string(j));
      return err_;
    }
  }

  robotParam_.collisionParam.enable = false;
  err_ = robot_->mKinematics.SetCollisionParam(
    robotParam_.robotParam.robotId, robotParam_.collisionParam);
  if (err_ != ErrorCode::None) {logErr(err_, "Failed to set collision param to engine"); return err_;}

  robotParam_.robotParam.toolLinkParam.linkOrigin = wmx3Api::coordinate::Point3d(0.0, 0.0, 0.1);
  robotParam_.robotParam.toolLinkParam.linkInertia.mass = 1.0;
  err_ = robot_->mKinematics.SetToolInfo(
    robotParam_.robotParam.robotId,
    robotParam_.robotParam.toolCoordinate,
    robotParam_.robotParam.toolLinkParam.linkInertia.mass,
    robotParam_.robotParam.toolLinkParam.linkOrigin);
  if (err_ != ErrorCode::None) {logErr(err_, "Failed to set tool info to engine"); return err_;}

  params_uploaded_ = true;
  RCLCPP_INFO(this->get_logger(), "Finished uploading robot parameters");
  return ErrorCode::None;
}

// Apply WMX3 system/axis parameters (port of configure_device).
int WmxRobotOptionNode::configureDevice()
{
  // When no parameter file is given, the WMX3 system/axis parameters are owned by
  // another node sharing the engine (e.g. joint_trajectory_controller, which calls
  // ImportAndSetAll itself). This option then just attaches to the already-configured
  // engine instead of re-importing — see wmx_ros2_robot_option.launch.py.
  if (wmx_param_file_path_.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "wmx_param_file_path empty; using WMX parameters configured by another node");
    device_configured_ = true;
    return ErrorCode::None;
  }

  err_ = robot_->mCoreMotion.config->ImportAndSetAll(
    const_cast<char *>(wmx_param_file_path_.c_str()));
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to import WMX parameters. Error=%d (%s)", err_, errString_);
    return err_;
  }
  device_configured_ = true;
  RCLCPP_INFO(this->get_logger(), "Device configured successfully");
  return ErrorCode::None;
}

void WmxRobotOptionNode::statusStep()
{
  robot_->UpdateRobotStatus(robotParam_.robotParam.robotId, robotStatus_);
  stopMotionIfAlarm();

  statusMsg_.header.stamp = this->now();
  statusMsg_.header.frame_id = status_frame_;
  statusMsg_.device = isReady();
  statusMsg_.servo = getServo();
  statusMsg_.in_motion = areArmAxesBusy();
  statusMsg_.is_pass_motion = false;
  statusMsg_.motion_state = motionStateString(robotStatus_.motionState);

  std::string motion_error;
  std::tie(motion_error, statusMsg_.motion_error_axis) = motionErrorInfo(robotStatus_.motionErrorInfo);
  statusMsg_.motion_error = motion_error;

  statusMsg_.cartesian_pose_fb = wmx_robot_option::pose_to_vector(robotStatus_.stateFeedback.toolPose);
  statusMsg_.cartesian_pose_cmd = wmx_robot_option::pose_to_vector(robotStatus_.stateCommand.toolPose);

  // Joint feedback/command are reported in radians (revolute) / metres (prismatic)
  // to match the srv convention, mirroring ArmThread::get_arm_angle_fb/cmd in
  // wmx_arm_server (the engine stores revolute joint positions in degrees).
  int n = robotParam_.robotParam.numJoints;
  statusMsg_.joint_angles_fb.resize(n);
  statusMsg_.joint_angles_cmd.resize(n);
  for (int i = 0; i < n; ++i) {
    double scale = isRevoluteJoint(i) ? wmx_robot_option::DEG2RAD : 1.0;
    statusMsg_.joint_angles_fb[i] = robotStatus_.stateFeedback.jointPosition[i] * scale;
    statusMsg_.joint_angles_cmd[i] = robotStatus_.stateCommand.jointPosition[i] * scale;
  }

  statusMsg_.light = false;
  statusMsg_.gripper = 0.0;
  statusPub_->publish(statusMsg_);
}

// ---------------------------------------------------------------------------
// Option helpers (port of WMX_Arm)
// ---------------------------------------------------------------------------

bool WmxRobotOptionNode::isReady()
{
  robot_->mCoreMotion.GetStatus(&cmStatus_);
  return cmStatus_.engineState == wmx3Api::EngineState::Communicating &&
         robot_->mCoreMotion.IsDeviceValid() &&
         params_uploaded_ && device_configured_;
}

bool WmxRobotOptionNode::getServo()
{
  if (!params_uploaded_) {
    return false;
  }
  robot_->mCoreMotion.GetStatus(&cmStatus_);
  for (int i = 0; i < robotParam_.robotParam.numJoints; ++i) {
    if (!cmStatus_.axesStatus[robotParam_.robotParam.jointParams[i].axis].servoOn) {
      return false;
    }
  }
  return true;
}

// True if any arm axis is still executing a motion. Reads the shared CoreMotion
// axis state (rather than this node's kinematics motionState), so it also sees
// motions started by other clients of the same engine (e.g.
// joint_trajectory_controller in wmx_ros2_package). It doubles as the cross-node
// interlock that keeps the two from commanding the arm at once and as the
// in_motion flag published on the status topic.
bool WmxRobotOptionNode::areArmAxesBusy()
{
  wmx3Api::CoreMotionStatus status;
  robot_->mCoreMotion.GetStatus(&status);
  for (int i = 0; i < robotParam_.robotParam.numJoints; ++i) {
    if (!status.axesStatus[robotParam_.robotParam.jointParams[i].axis].inPos) {
      return true;
    }
  }
  return false;
}

bool WmxRobotOptionNode::isAxesNormalState()
{
  wmx3Api::CoreMotionStatus status;
  robot_->mCoreMotion.GetStatus(&status);
  for (int i = 0; i < robotParam_.robotParam.numJoints; ++i) {
    if (status.axesStatus[robotParam_.robotParam.jointParams[i].axis].ampAlarm) {
      return false;
    }
  }
  return true;
}

bool WmxRobotOptionNode::isRevoluteJoint(int joint_index)
{
  auto type = robotParam_.robotParam.jointParams[joint_index].jointType;
  return type >= wmx3Api::kinematics::JointParam::JointType::RotateXAxis &&
         type <= wmx3Api::kinematics::JointParam::JointType::RotateZAxis;
}

void WmxRobotOptionNode::stopMotionIfAlarm()
{
  if (!isAxesNormalState() && areArmAxesBusy()) {
    RCLCPP_ERROR(this->get_logger(), "Axis state is not normal, stopping motion");
    robot_->mKinematics.StopMotion(robotParam_.robotParam.robotId);
  }
}

std::string WmxRobotOptionNode::motionStateString(MotionState state)
{
  switch (state) {
    case MotionState::RobotMotionIdle: return "idle";
    case MotionState::RobotMotionNoNewCommand: return "no_new_command";
    case MotionState::RobotMotionStop: return "cp_stop";
    case MotionState::RobotMotionStopPTP: return "ptp_stop";
    case MotionState::RobotMotionEStop: return "estop";
    case MotionState::RobotMotionSetup: return "setup";
    case MotionState::RobotMotionInMotion: return "cp_in_motion";
    case MotionState::RobotMotionInPTPMotion: return "ptp_in_motion";
    case MotionState::RobotMotionInErrorStopping: return "ptp_in_error_stop";
    case MotionState::RobotMotionInErrorStoppingCP: return "cp_in_error_stop";
    case MotionState::RobotMotionError: return "error";
    case MotionState::RobotMotionInPausing: return "cp_in_pausing";
    case MotionState::RobotMotionInPTPPausing: return "ptp_in_pausing";
    case MotionState::RobotMotionPaused: return "cp_in_paused";
    case MotionState::RobotMotionInPTPPaused: return "ptp_in_paused";
    default: return "undefined";
  }
}

std::tuple<std::string, int> WmxRobotOptionNode::motionErrorInfo(
  const wmx3Api::kinematics::MotionErrorInfo & info)
{
  switch (info.errorCode) {
    case MotionErrorCode::None: return {"none", -1};
    case MotionErrorCode::SystemEStop: return {"estop", -1};
    case MotionErrorCode::TrajectoryCalcError: return {"trajectory_calc_error", -1};
    case MotionErrorCode::InverseKinematicsError: return {"inverse_kinematics_error", -1};
    case MotionErrorCode::AxisPosLimitOver: return {"axis_position_limit_over", info.errorJoint + 1};
    case MotionErrorCode::AxisVelLimitOver: return {"axis_velocity_limit_over", info.errorJoint + 1};
    case MotionErrorCode::AxisAccLimitOver:
      return {"axis_acceleration_limit_over", info.errorJoint + 1};
    case MotionErrorCode::AxisAmpError: return {"axis_amp_error", -1};
    case MotionErrorCode::ToolRangeError: return {"tool_range_error", -1};
    case MotionErrorCode::ServoOff: return {"servo_off", -1};
    case MotionErrorCode::UserEStop: return {"user_estop", -1};
    case MotionErrorCode::AxisFollowingError: return {"axis_following_error", -1};
    case MotionErrorCode::AxisOffline: return {"axis_offline", -1};
    case MotionErrorCode::AxisInterruptMissmatch: return {"axis_interrupt_mismatch", -1};
    case MotionErrorCode::CollisionEStop: return {"collision_detected", info.errorJoint + 1};
    default: return {"undefined", -1};
  }
}

void WmxRobotOptionNode::logErr(int err_code, const std::string & msg)
{
  char err_string[256];
  if (err_code >= 0x100 && err_code <= 0x632) {
    wmx3Lib_.ErrorToString(err_code, err_string, sizeof(err_string));
  } else if (err_code >= 0x10000 && err_code <= 0x1007F) {
    robot_->mCoreMotion.ErrorToString(err_code, err_string, sizeof(err_string));
  } else if (err_code >= 0x26000 && err_code <= 0x2600D) {
    robot_->mCoordinate.ErrorToString(err_code, err_string, sizeof(err_string));
  } else if (err_code >= 0x28000 && err_code <= 0x28021) {
    robot_->mKinematics.ErrorToString(err_code, err_string, sizeof(err_string));
  } else {
    std::strncpy(err_string, "Unknown", sizeof(err_string));
  }
  RCLCPP_ERROR(this->get_logger(), "%s: %s (0x%X)", msg.c_str(), err_string, err_code);
}

// ---------------------------------------------------------------------------
// Motion primitives (port of WMX_Arm)
// ---------------------------------------------------------------------------

int WmxRobotOptionNode::setMotion(
  const wmx3Api::coordinate::CartesianPose & pose, bool is_tool_frame,
  int motion_type, bool start_motion)
{
  int err = ErrorCode::None;
  if (!is_tool_frame && checkTargetPose(pose)) {
    RCLCPP_INFO(this->get_logger(), "Robot already at pose, not moving");
    return ErrorCode::None;
  }

  switch (motion_type) {
    case MOTION_JOINT: {
        if (is_tool_frame) {
          wmx3Api::PtpMotionParam::PtpMovParam ptpParam(
            robotParam_.robotParam, robotParam_.profile);
          err = robot_->mKinematics.StartToolPTPMov(ptpParam, pose);
        } else {
          wmx3Api::PtpMotionParam::PtpPosParam ptpParam(
            robotParam_.robotParam, robotParam_.profile);
          err = robot_->mKinematics.StartPTPPos(ptpParam, pose);
        }
        if (err != ErrorCode::None) {logErr(err, "Failed to set PTP motion");}
      }
      break;

    case MOTION_LINEAR: {
        wmx3Api::TrajectoryMotionParam::TrajectoryLineMotionParam trajectoryParam(
          robotParam_.profile, pose, is_tool_frame);
        err = robot_->mKinematics.SetMotion(robotParam_.robotParam.robotId, trajectoryParam);
        if (err == wmx3Api::coordinate::CoordinateErrorCode::ZeroLengthTrajectory) {
          RCLCPP_INFO(this->get_logger(), "Line motion 'ZeroLengthTrajectory', skipping");
          return ErrorCode::None;
        } else if (err != ErrorCode::None) {
          logErr(err, "Failed to set line motion");
        }
      }
      break;

    default:
      RCLCPP_WARN(this->get_logger(), "Unsupported motion type: %d", motion_type);
      err = -1;
      break;
  }

  if (err == ErrorCode::None && motion_type != MOTION_JOINT && start_motion) {
    err = robot_->mKinematics.StartMotion(robotParam_.robotParam.robotId);
    if (err != ErrorCode::None) {logErr(err, "Failed to start motion");}
  }
  return err;
}

int WmxRobotOptionNode::cartesianMove(
  const wmx3Api::coordinate::CartesianPose & pose, bool is_tool_frame, int motion_type)
{
  if (!isAxesNormalState()) {
    RCLCPP_ERROR(this->get_logger(), "Axis state is not normal, refusing motion");
    return -1;
  }
  if (areArmAxesBusy()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Arm axes are busy (motion in progress, possibly from joint_trajectory_controller); "
      "refusing motion");
    return -1;
  }
  return setMotion(pose, is_tool_frame, motion_type, true);
}

int WmxRobotOptionNode::jointMove(const std::vector<double> & angles)
{
  if (checkTargetAngle(angles)) {
    return ErrorCode::None;
  }
  if (!isAxesNormalState()) {
    RCLCPP_ERROR(this->get_logger(), "Axis state is not normal, refusing motion");
    return -1;
  }
  if (areArmAxesBusy()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Arm axes are busy (motion in progress, possibly from joint_trajectory_controller); "
      "refusing motion");
    return -1;
  }

  double target[wmx3Api::kinematics::constants::MAX_NUMBER_OF_JOINT] = {0.0};
  for (int i = 0; i < robotParam_.robotParam.numJoints; ++i) {
    target[i] = angles[i];
  }
  wmx3Api::PtpMotionParam::PtpPosParam ptpParam(
    robotParam_.robotParam, robotParam_.profile, target);
  int err = robot_->mKinematics.StartPTPPos(ptpParam);
  if (err != ErrorCode::None) {logErr(err, "Failed to set PTP joint motion");}
  return err;
}

int WmxRobotOptionNode::jointRelMove(const std::vector<double> & angles)
{
  if (checkTargetAngle(angles)) {
    return ErrorCode::None;
  }
  if (!isAxesNormalState()) {
    RCLCPP_ERROR(this->get_logger(), "Axis state is not normal, refusing motion");
    return -1;
  }
  if (areArmAxesBusy()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Arm axes are busy (motion in progress, possibly from joint_trajectory_controller); "
      "refusing motion");
    return -1;
  }

  double target[wmx3Api::kinematics::constants::MAX_NUMBER_OF_JOINT] = {0.0};
  for (int i = 0; i < robotParam_.robotParam.numJoints; ++i) {
    target[i] = angles[i];
  }
  wmx3Api::PtpMotionParam::PtpMovParam ptpParam(
    robotParam_.robotParam, robotParam_.profile, target);
  int err = robot_->mKinematics.StartPTPPos(ptpParam);
  if (err != ErrorCode::None) {logErr(err, "Failed to set relative PTP joint motion");}
  return err;
}

bool WmxRobotOptionNode::checkTargetPose(
  const wmx3Api::coordinate::CartesianPose & pose, double tolerance)
{
  robot_->UpdateRobotStatus(robotParam_.robotParam.robotId, robotStatus_);
  std::vector<double> cur = wmx_robot_option::pose_to_vector(robotStatus_.stateFeedback.toolPose);
  std::vector<double> tgt = wmx_robot_option::pose_to_vector(pose);

  auto close = [tolerance](double a, double b) {return std::fabs(a - b) < tolerance;};
  bool translation = close(cur[0], tgt[0]) && close(cur[1], tgt[1]) && close(cur[2], tgt[2]);
  // Quaternions with all signs flipped describe the same orientation.
  bool same = close(cur[3], tgt[3]) && close(cur[4], tgt[4]) &&
    close(cur[5], tgt[5]) && close(cur[6], tgt[6]);
  bool flipped = close(cur[3], -tgt[3]) && close(cur[4], -tgt[4]) &&
    close(cur[5], -tgt[5]) && close(cur[6], -tgt[6]);
  return translation && (same || flipped);
}

bool WmxRobotOptionNode::checkTargetAngle(const std::vector<double> & angles, double tolerance)
{
  robot_->UpdateRobotStatus(robotParam_.robotParam.robotId, robotStatus_);
  for (size_t i = 0; i < angles.size(); ++i) {
    if (std::fabs(angles[i] - robotStatus_.stateFeedback.jointPosition[i]) >= tolerance) {
      return false;
    }
  }
  return true;
}

// Enable collision detection ahead of a motion (port of set_collision_enable(true)).
// Honors the same external output-bit gate as wmx_arm_server.
void WmxRobotOptionNode::enableCollisionForMotion()
{
  unsigned char gate = 0;
  io_->GetOutBitEx(0, 0, &gate);
  if (gate == 0) {
    return;
  }
  wmx3Api::kinematics::CollisionParam collisionParam;
  if (robot_->mKinematics.GetCollisionParam(
      robotParam_.robotParam.robotId, collisionParam) != ErrorCode::None)
  {
    return;
  }
  collisionParam.enable = true;
  robot_->mKinematics.SetCollisionParam(robotParam_.robotParam.robotId, collisionParam);
}

// ---------------------------------------------------------------------------
// Service callbacks
// ---------------------------------------------------------------------------

void WmxRobotOptionNode::setServo(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }
  wmx3Api::AxisSelection axisSel = robot_->mRobotConfig.GetAxisSelection(robotParam_.robotParam);
  err_ = robot_->mCoreMotion.axisControl->SetServoOn(&axisSel, request->data);
  if (err_ != ErrorCode::None) {
    logErr(err_, "Failed to set servo");
    response->success = false;
    response->message = "Failed to set servo";
    return;
  }
  response->success = true;
  response->message = request->data ? "Servo on" : "Servo off";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void WmxRobotOptionNode::setSpeed(
  const std::shared_ptr<wmx_ros2_message::srv::SetRobotScalar::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::SetRobotScalar::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }
  double gain = request->value * 0.01;
  gain = std::clamp(gain, 0.01, 1.0);
  robotParam_.profile.velOverride = gain;
  robotParam_.profile.accOverride = gain;
  response->success = true;
  snprintf(buffer_, sizeof(buffer_), "Speed set to %.1f%%", gain * 100.0);
  response->message = buffer_;
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
}

void WmxRobotOptionNode::clearErrors(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }
  robot_->mCoreMotion.GetStatus(&cmStatus_);
  for (int i = 0; i < robotParam_.robotParam.numJoints; ++i) {
    int axis = robotParam_.robotParam.jointParams[i].axis;
    robot_->mCoreMotion.axisControl->ClearAmpAlarm(axis);
    robot_->mCoreMotion.axisControl->ClearAxisAlarm(axis);
  }
  err_ = robot_->mKinematics.ClearMotionError(robotParam_.robotParam.robotId);
  if (err_ != ErrorCode::None) {
    logErr(err_, "Failed to clear motion errors");
    response->success = false;
    response->message = "Failed to clear motion errors";
    return;
  }
  response->success = true;
  response->message = "Errors cleared";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void WmxRobotOptionNode::stopMotion(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }
  err_ = robot_->mKinematics.StopMotion(robotParam_.robotParam.robotId);
  if (err_ != ErrorCode::None) {
    logErr(err_, "Failed to stop motion");
    response->success = false;
    response->message = "Failed to stop motion";
    return;
  }
  response->success = true;
  response->message = "Motion stopped";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void WmxRobotOptionNode::exportParams(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }

  std::string export_path;
  this->get_parameter("robot_export_xml_path", export_path);
  if (export_path.empty()) {
    export_path = robot_xml_path_ + "_export.xml";
  }

  wmx3Api::RobotMotionParam temp = robotParam_;
  err_ = robot_->mKinematics.GetRobotParam(temp.robotParam.robotId, temp.robotParam);
  if (err_ != ErrorCode::None) {logErr(err_, "Failed to get robot param");}
  for (int j = 0; j < temp.robotParam.numJoints && err_ == ErrorCode::None; ++j) {
    err_ = robot_->mKinematics.GetFittingData(temp.robotParam.robotId, j, temp.fittingData[j]);
  }
  if (err_ == ErrorCode::None) {
    err_ = robot_->mKinematics.GetCollisionParam(temp.robotParam.robotId, temp.collisionParam);
  }
  if (err_ == ErrorCode::None) {
    err_ = robot_->mRobotConfig.ExportParamXML(const_cast<char *>(export_path.c_str()), temp);
  }

  if (err_ != ErrorCode::None) {
    logErr(err_, "Failed to export robot params");
    response->success = false;
    response->message = "Failed to export robot params";
    return;
  }
  response->success = true;
  response->message = "Exported robot params to: " + export_path;
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void WmxRobotOptionNode::jogPose(
  const std::shared_ptr<wmx_ros2_message::srv::RobotJogPose::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::RobotJogPose::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    response->error_code = -1;
    return;
  }
  if (request->offset.size() != 6) {
    response->success = false;
    response->message = "offset must have 6 elements [x, y, z, rx, ry, rz]";
    response->error_code = -1;
    return;
  }

  robot_->UpdateRobotStatus(robotParam_.robotParam.robotId, robotStatus_);
  wmx3Api::coordinate::CartesianPose current = robotStatus_.stateFeedback.toolPose;
  wmx3Api::coordinate::CartesianPose offset = wmx_robot_option::offset_pose(
    wmx3Api::coordinate::CartesianPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), request->offset);

  wmx3Api::coordinate::CartesianPose return_pose;
  wmx3Api::coordinate::CartesianPose pose_to_set;
  if (request->is_tool_frame) {
    return_pose = current;
    robot_->mCoordinate.AddInToolCoordinate(return_pose, offset);
    pose_to_set = offset;
  } else {
    return_pose = current;
    robot_->mCoordinate.AddInWorkCoordinate(return_pose, offset);
    pose_to_set = return_pose;
  }

  enableCollisionForMotion();
  response->error_code = cartesianMove(pose_to_set, request->is_tool_frame, MOTION_LINEAR);
  response->success = (response->error_code == ErrorCode::None);
  response->result_pose = wmx_robot_option::pose_to_vector(return_pose);
  response->message = response->success ? "Jog pose accepted" : "Jog pose failed";
}

void WmxRobotOptionNode::jogPoseAbsolute(
  const std::shared_ptr<wmx_ros2_message::srv::RobotJogPose::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::RobotJogPose::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    response->error_code = -1;
    return;
  }
  if (request->offset.size() != 6) {
    response->success = false;
    response->message = "offset must have 6 elements [x, y, z, rx, ry, rz]";
    response->error_code = -1;
    return;
  }

  robot_->UpdateRobotStatus(robotParam_.robotParam.robotId, robotStatus_);
  wmx3Api::coordinate::CartesianPose absolute = wmx_robot_option::absolute_pose(
    robotStatus_.stateFeedback.toolPose, request->offset);

  enableCollisionForMotion();
  response->error_code = cartesianMove(absolute, false, MOTION_LINEAR);
  response->success = (response->error_code == ErrorCode::None);
  response->result_pose = wmx_robot_option::pose_to_vector(absolute);
  response->message = response->success ? "Jog pose absolute accepted" : "Jog pose absolute failed";
}

void WmxRobotOptionNode::setPosePtp(
  const std::shared_ptr<wmx_ros2_message::srv::RobotMovePose::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::RobotMovePose::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    response->error_code = -1;
    return;
  }
  if (request->pose.size() != 7) {
    response->success = false;
    response->message = "pose must have 7 elements [x, y, z, qx, qy, qz, qw]";
    response->error_code = -1;
    return;
  }

  wmx3Api::coordinate::CartesianPose pose = wmx_robot_option::vector_to_pose(request->pose);
  enableCollisionForMotion();
  response->error_code = cartesianMove(pose, request->is_tool_frame, request->motion_type);
  response->success = (response->error_code == ErrorCode::None);
  response->result_pose = request->pose;
  response->message = response->success ? "Set pose accepted" : "Set pose failed";
}

void WmxRobotOptionNode::jogAngle(
  const std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    response->error_code = -1;
    return;
  }

  // Convert the radian/metre request into the SDK's degree/metre convention.
  std::vector<double> offset(request->angle.size(), 0.0);
  for (size_t i = 0; i < offset.size(); ++i) {
    offset[i] = isRevoluteJoint(i) ? request->angle[i] * RAD2DEG : request->angle[i];
  }

  enableCollisionForMotion();
  response->error_code = jointRelMove(offset);
  response->success = (response->error_code == ErrorCode::None);
  response->result_angle = request->angle;
  response->message = response->success ? "Jog angle accepted" : "Jog angle failed";
}

void WmxRobotOptionNode::jogAngleAbsolute(
  const std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    response->error_code = -1;
    return;
  }

  robot_->UpdateRobotStatus(robotParam_.robotParam.robotId, robotStatus_);
  std::vector<double> absolute(request->angle.size(), 0.0);
  std::vector<double> result(request->angle.size(), 0.0);
  for (size_t i = 0; i < absolute.size(); ++i) {
    double current = robotStatus_.stateFeedback.jointPosition[i];  // deg or mm
    double requested = isRevoluteJoint(i) ? request->angle[i] * RAD2DEG : request->angle[i];
    absolute[i] = wmx_robot_option::replace_nonzero(current, requested);
    result[i] = isRevoluteJoint(i) ? absolute[i] * wmx_robot_option::DEG2RAD : absolute[i];
  }

  enableCollisionForMotion();
  response->error_code = jointMove(absolute);
  response->success = (response->error_code == ErrorCode::None);
  response->result_angle = result;
  response->message = response->success ? "Jog angle absolute accepted" : "Jog angle absolute failed";
}

void WmxRobotOptionNode::setAnglePtp(
  const std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    response->error_code = -1;
    return;
  }

  std::vector<double> target(request->angle.size(), 0.0);
  for (size_t i = 0; i < target.size(); ++i) {
    target[i] = isRevoluteJoint(i) ? request->angle[i] * RAD2DEG : request->angle[i];
  }

  enableCollisionForMotion();
  response->error_code = jointMove(target);
  response->success = (response->error_code == ErrorCode::None);
  response->result_angle = request->angle;
  response->message = response->success ? "Set angle accepted" : "Set angle failed";
}

void WmxRobotOptionNode::checkPose(
  const std::shared_ptr<wmx_ros2_message::srv::RobotCheckPose::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::RobotCheckPose::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->reachable = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }
  if (request->pose.size() != 7) {
    response->success = false;
    response->reachable = false;
    response->message = "pose must have 7 elements [x, y, z, qx, qy, qz, qw]";
    return;
  }
  wmx3Api::coordinate::CartesianPose pose = wmx_robot_option::vector_to_pose(request->pose);
  response->reachable = checkTargetPose(pose);
  response->success = true;
  response->message = response->reachable ? "Pose is a valid target" : "Pose not reached / mismatch";
}

void WmxRobotOptionNode::checkAngle(
  const std::shared_ptr<wmx_ros2_message::srv::RobotCheckAngle::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::RobotCheckAngle::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->reachable = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }
  std::vector<double> target(request->angle.size(), 0.0);
  for (size_t i = 0; i < target.size(); ++i) {
    target[i] = isRevoluteJoint(i) ? request->angle[i] * RAD2DEG : request->angle[i];
  }
  response->reachable = checkTargetAngle(target);
  response->success = true;
  response->message = response->reachable ? "Angle is a valid target" : "Angle mismatch";
}

void WmxRobotOptionNode::setCollisionEnable(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }

  // Collision detection is gated by an external output bit (mirrors check_output()
  // in wmx_arm_server): when the bit is low, detection stays disabled.
  unsigned char gate = 0;
  io_->GetOutBitEx(0, 0, &gate);
  if (gate == 0) {
    response->success = true;
    response->message = "Collision detection gated off by output bit";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  wmx3Api::kinematics::CollisionParam collisionParam;
  err_ = robot_->mKinematics.GetCollisionParam(robotParam_.robotParam.robotId, collisionParam);
  if (err_ == ErrorCode::None) {
    collisionParam.enable = request->data;
    err_ = robot_->mKinematics.SetCollisionParam(robotParam_.robotParam.robotId, collisionParam);
  }
  if (err_ != ErrorCode::None) {
    logErr(err_, "Failed to set collision enable");
    response->success = false;
    response->message = "Failed to set collision enable";
    return;
  }
  response->success = true;
  response->message = request->data ? "Collision detection enabled" : "Collision detection disabled";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void WmxRobotOptionNode::setFittingParam(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }

  if (request->data) {
    err_ = robot_->mKinematics.SetFittingOn(robotParam_.robotParam.robotId, true);
    if (err_ != ErrorCode::None) {
      logErr(err_, "Failed to enable fitting measurement");
      response->success = false;
      response->message = "Failed to enable fitting measurement";
      return;
    }
    response->success = true;
    response->message = "Fitting measurement enabled";
    return;
  }

  err_ = robot_->mKinematics.CalcFittingData(robotParam_.robotParam.robotId);
  if (err_ != ErrorCode::None) {
    logErr(err_, "Failed to calculate fitting parameters");
    response->success = false;
    response->message = "Failed to calculate fitting parameters";
    return;
  }
  err_ = robot_->mKinematics.SetFittingOn(robotParam_.robotParam.robotId, false);
  if (err_ != ErrorCode::None) {
    logErr(err_, "Failed to disable fitting measurement");
    response->success = false;
    response->message = "Failed to disable fitting measurement";
    return;
  }
  response->success = true;
  response->message = "Fitting parameters calculated and measurement disabled";
}

void WmxRobotOptionNode::setCollisionSensitivity(
  const std::shared_ptr<wmx_ros2_message::srv::SetRobotScalar::Request> request,
  std::shared_ptr<wmx_ros2_message::srv::SetRobotScalar::Response> response)
{
  if (!initialized_) {
    response->success = false;
    response->message = "Robot option not initialized. Engine not ready.";
    return;
  }
  wmx3Api::kinematics::CollisionParam collisionParam;
  err_ = robot_->mKinematics.GetCollisionParam(robotParam_.robotParam.robotId, collisionParam);
  if (err_ == ErrorCode::None) {
    collisionParam.thresHold = request->value;
    err_ = robot_->mKinematics.SetCollisionParam(robotParam_.robotParam.robotId, collisionParam);
  }
  if (err_ != ErrorCode::None) {
    logErr(err_, "Failed to set collision sensitivity");
    response->success = false;
    response->message = "Failed to set collision sensitivity";
    return;
  }
  collision_sensitivity_ = request->value;
  snprintf(buffer_, sizeof(buffer_), "Collision sensitivity set to %.3f", request->value);
  response->success = true;
  response->message = buffer_;
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxRobotOptionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
