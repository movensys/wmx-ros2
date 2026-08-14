// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef GRIPPER_CONTROLLER_HPP_
#define GRIPPER_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "WMX3Api.h"
#include "IOApi.h"

class GripperControllerApi
{
public:
  explicit GripperControllerApi(const rclcpp::Logger & logger);
  ~GripperControllerApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  int setOutputBit(int32_t byte, int32_t bit, int32_t value, std::string & message);
  int setOutputByte(int32_t byte, int32_t value, std::string & message);
  int getOutputByte(int32_t byte, int32_t & value, std::string & message);
  int getInputBit(int32_t byte, int32_t bit, int32_t & value, std::string & message);

  bool isDeviceOpen() const {return isDeviceAttached_;}

private:
  rclcpp::Logger logger_;

  const char * deviceName_ = "gripper_controller";
  unsigned int timeout_ = 10000;

  mutable std::mutex deviceMutex_;
  bool isDeviceAttached_ = false;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::IO io_;
};

class GripperController : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  GripperController();
  ~GripperController() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<GripperControllerApi> api_;

  std::vector<int64_t> gripperAddress_;
  std::string wmxGripperTopic_;

  std::atomic<bool> isNodeActive_{false};

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setGripperService_;

  bool isNodeActive() const;
  std::string notActiveMessage() const;

  void setRosParameter();

  void dobotCR3AGripperSetup();

  void setGripperCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

#endif  // GRIPPER_CONTROLLER_HPP_
