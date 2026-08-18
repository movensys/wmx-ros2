// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// servo_stream_test_source
//
// Synthetic stand-in for MoveIt Servo. Publishes trajectory_msgs/JointTrajectory
// on Servo's command_out_topic at Servo's rate, so the WMX3 streaming path can be
// characterised on one axis without MoveIt, IK, collision checking, or a
// teleop device in the loop.
//
// Why bother: with Servo in the loop you cannot tell whether a tracking error
// came from the API buffer, from IK, from the smoothing filter, or from the
// operator's hand. Here the input is a known analytic function, so commanded-vs-
// actual is a direct measurement of the streaming path alone.
//
// It publishes ONLY the joints named in joint_name, which also exercises the
// subset path in servo_stream_controller (a real six-joint Servo message has
// the same shape from the controller's point of view).
//
// The waveform is centred on the joint's CURRENT position, read from
// /joint_states, and the amplitude ramps in from zero — so starting the node
// never produces a step command.
//
// Setting amplitude_rad: 0.0 gives a pure hold: the full streaming path runs at
// rate with zero commanded motion. That is the correct first test with the
// servo on, before commanding any movement at all.
//
// Stopping this node (Ctrl-C) simply stops the stream, which is the intended way
// to exercise servo_stream_controller's starvation path.

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using std::placeholders::_1;

namespace
{
// A bench test should not be able to command a large excursion by typo.
constexpr double kAmplitudeHardCapRad = 0.5;
}  // namespace

class ServoStreamTestSource : public rclcpp::Node
{
public:
  ServoStreamTestSource();

private:
  std::vector<std::string> jointNames_;
  std::string jointStatesTopic_;
  std::string outputTopic_;
  double amplitudeRad_ = 0.0;
  double frequencyHz_ = 0.25;
  double publishRateHz_ = 40.0;
  double rampSeconds_ = 2.0;
  std::string waveform_ = "sine";

  std::mutex mtx_;
  std::map<std::string, double> currentPos_;
  std::vector<double> center_;
  bool centered_ = false;
  double phase_ = 0.0;
  double elapsed_ = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmdPub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void setRosParameter();
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool captureCenter();
  void tick();
};

ServoStreamTestSource::ServoStreamTestSource()
: Node("servo_stream_test_source")
{
  setRosParameter();

  cmdPub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(outputTopic_, 1);

  jointStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    jointStatesTopic_, rclcpp::SensorDataQoS(),
    std::bind(&ServoStreamTestSource::onJointState, this, _1));

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publishRateHz_));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ServoStreamTestSource::tick, this));

  RCLCPP_INFO(
    this->get_logger(), "servo_stream_test_source waiting for %s ...",
    jointStatesTopic_.c_str());
}

void ServoStreamTestSource::setRosParameter()
{
  this->declare_parameter<std::vector<std::string>>("joint_name", {"joint1"});
  this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
  this->declare_parameter<std::string>(
    "output_topic", "/movensys_manipulator_arm_controller/joint_trajectory");
  this->declare_parameter<double>("amplitude_rad", 0.0);
  this->declare_parameter<double>("frequency_hz", 0.25);
  this->declare_parameter<double>("publish_rate_hz", 40.0);
  this->declare_parameter<double>("ramp_seconds", 2.0);
  this->declare_parameter<std::string>("waveform", "sine");

  this->get_parameter("joint_name", jointNames_);
  this->get_parameter("joint_states_topic", jointStatesTopic_);
  this->get_parameter("output_topic", outputTopic_);
  this->get_parameter("amplitude_rad", amplitudeRad_);
  this->get_parameter("frequency_hz", frequencyHz_);
  this->get_parameter("publish_rate_hz", publishRateHz_);
  this->get_parameter("ramp_seconds", rampSeconds_);
  this->get_parameter("waveform", waveform_);

  if (std::fabs(amplitudeRad_) > kAmplitudeHardCapRad) {
    RCLCPP_WARN(
      this->get_logger(), "amplitude_rad %.3f exceeds the %.2f rad bench cap; clamping",
      amplitudeRad_, kAmplitudeHardCapRad);
    amplitudeRad_ = std::copysign(kAmplitudeHardCapRad, amplitudeRad_);
  }
  if (waveform_ != "sine" && waveform_ != "triangle") {
    RCLCPP_WARN(this->get_logger(), "unknown waveform '%s'; using sine", waveform_.c_str());
    waveform_ = "sine";
  }

  std::string names;
  for (size_t i = 0; i < jointNames_.size(); ++i) {
    if (i > 0) {names += ", ";}
    names += jointNames_[i];
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_name: [%s]", names.c_str());
  RCLCPP_INFO(this->get_logger(), "output_topic: %s", outputTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "amplitude_rad: %.4f (%s)", amplitudeRad_,
    amplitudeRad_ == 0.0 ? "HOLD — no commanded motion" : waveform_.c_str());
  RCLCPP_INFO(this->get_logger(), "frequency_hz: %.3f", frequencyHz_);
  RCLCPP_INFO(this->get_logger(), "publish_rate_hz: %.1f", publishRateHz_);
  RCLCPP_INFO(this->get_logger(), "ramp_seconds: %.1f", rampSeconds_);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void ServoStreamTestSource::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    currentPos_[msg->name[i]] = msg->position[i];
  }
}

// Latch the waveform centre on the joints' present positions, so the first
// command published equals where the arm already is.
bool ServoStreamTestSource::captureCenter()
{
  std::lock_guard<std::mutex> lock(mtx_);
  std::vector<double> centre(jointNames_.size(), 0.0);

  for (size_t i = 0; i < jointNames_.size(); ++i) {
    const auto it = currentPos_.find(jointNames_[i]);
    if (it == currentPos_.end()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for '%s' on %s ...", jointNames_[i].c_str(), jointStatesTopic_.c_str());
      return false;
    }
    centre[i] = it->second;
  }

  center_ = centre;
  centered_ = true;

  std::string vals;
  for (size_t i = 0; i < center_.size(); ++i) {
    if (i > 0) {vals += ", ";}
    vals += std::to_string(center_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "Centred on [%s]; streaming.", vals.c_str());
  return true;
}

void ServoStreamTestSource::tick()
{
  if (!centered_ && !captureCenter()) {
    return;
  }

  const double dt = 1.0 / std::max(1.0, publishRateHz_);
  elapsed_ += dt;
  phase_ += 2.0 * M_PI * frequencyHz_ * dt;
  if (phase_ > 2.0 * M_PI) {
    phase_ -= 2.0 * M_PI;
  }

  // Ramp the amplitude in so starting the node is never a step command.
  const double ramp =
    (rampSeconds_ > 0.0) ? std::min(1.0, elapsed_ / rampSeconds_) : 1.0;

  double shape = 0.0;
  if (waveform_ == "sine") {
    shape = std::sin(phase_);
  } else {
    // Triangle: constant velocity except at the turnarounds — a harsher test of
    // the blend, since velocity reverses instantly at each peak.
    const double t = phase_ / (2.0 * M_PI);
    shape = (t < 0.5) ? (4.0 * t - 1.0) : (3.0 - 4.0 * t);
  }

  const double offset = amplitudeRad_ * ramp * shape;

  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = this->now();
  msg.joint_names = jointNames_;

  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions.resize(jointNames_.size());
  for (size_t i = 0; i < jointNames_.size(); ++i) {
    pt.positions[i] = center_[i] + offset;
  }
  // joint_position_controller derives its velocity from time_from_start, so set
  // it to the publish period to keep the A/B comparison fair.
  pt.time_from_start = rclcpp::Duration::from_seconds(dt);
  msg.points.push_back(pt);

  cmdPub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoStreamTestSource>());
  rclcpp::shutdown();
  return 0;
}
