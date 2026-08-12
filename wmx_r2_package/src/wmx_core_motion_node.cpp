// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_core_motion_node.hpp"

using wmx3Api::AxisCommandMode;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;

WmxCoreMotionNode::WmxCoreMotionNode()
: LifecycleNode("wmx_core_motion_node")
{
  jogTimeoutMs_ = this->declare_parameter("jog_timeout_ms", 200.0);
  jogRunTimeMs_ = this->declare_parameter("jog_run_time_ms", 2000.0);
  jogJerkRatio_ = this->declare_parameter("jog_jerk_ratio", 0.75);

  homing_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_INFO(
    this->get_logger(), "wmx_core_motion_node is unconfigured, waiting for configure...");
}

WmxCoreMotionNode::~WmxCoreMotionNode()
{
  releaseDevice();
  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node stopped");
}

std::string WmxCoreMotionNode::notActiveMessage() const
{
  return "wmx_core_motion_node is not active (state: " +
         this->get_current_state().label() + ").";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxCoreMotionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
