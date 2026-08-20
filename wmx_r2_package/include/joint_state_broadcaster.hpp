// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef JOINT_STATE_BROADCASTER_HPP_
#define JOINT_STATE_BROADCASTER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "wmx_r2_message/srv/set_axes.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "IOApi.h"

class JointStateBroadcasterApi
{
public:
  struct AxisFeedback
  {
    double actualPos = 0.0;
    double actualVelocity = 0.0;
  };

  explicit JointStateBroadcasterApi(const rclcpp::Logger & logger);
  ~JointStateBroadcasterApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  int getStatus(
    const std::vector<int64_t> & axes, std::vector<AxisFeedback> & feedback,
    std::string & message);

  int getOutputBit(int32_t byte, int32_t bit, int32_t & value, std::string & message);

  int setServoOn(int axis, int on, std::string & message);

  bool isDeviceOpen() const {return cm_ != nullptr;}

private:
  std::string errorText(int err);

  rclcpp::Logger logger_;

  const char * deviceName_ = "joint_state_broadcaster";
  unsigned int timeout_ = 10000;

  std::mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::CoreMotion> cm_;
  std::unique_ptr<wmx3Api::IO> io_;
};

class JointStateBroadcaster : public rclcpp::Node
{
public:
  JointStateBroadcaster();
  ~JointStateBroadcaster();

private:
  std::unique_ptr<JointStateBroadcasterApi> api_;

  int jointFeedbackRate_;
  float gripperCloseValue_;
  float gripperOpenValue_;
  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> gripperJointNames_;
  std::vector<int64_t> gripperAddress_;
  std::string encoderJointTopic_;
  std::string isaacsimJointTopic_;
  std::string gazeboJointTopic_;

  bool initialized_ = false;
  std::atomic<bool> initializing_{false};

  static constexpr int kMaxDeviceRetries = 30;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr coreMotionReadySub_;
  rclcpp::Client<wmx_r2_message::srv::SetAxes>::SharedPtr clearAmpAlarmClient_;
  rclcpp::Client<wmx_r2_message::srv::SetAxes>::SharedPtr setServoOnClient_;

  rclcpp::TimerBase::SharedPtr encoderJointTimer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoderJointPub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr isaacsimJointPub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gazeboJointPub_;

  std::thread init_thread_;

  void onCoreMotionReady(const std_msgs::msg::Bool::SharedPtr msg);
  void runInitSequence();
  bool attachDeviceWithRetries();
  bool callSetAxesService(
    rclcpp::Client<wmx_r2_message::srv::SetAxes>::SharedPtr client,
    const std::string & service_name,
    const std::vector<int64_t> & index,
    const std::vector<int64_t> & data);
  void servoOff();
  void publishJointState();
  void setRosParameter();
};

#endif  // JOINT_STATE_BROADCASTER_HPP_
