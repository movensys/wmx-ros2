// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_io_node.hpp"

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

WmxIoNode::WmxIoNode()
: LifecycleNode("wmx_io_node")
{
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is unconfigured, waiting for configure...");
}

WmxIoNode::~WmxIoNode()
{
  releaseDevice();
  RCLCPP_INFO(this->get_logger(), "wmx_io_node stopped");
}

std::string WmxIoNode::notActiveMessage() const
{
  return "wmx_io_node is not active (state: " +
         this->get_current_state().label() + ").";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxIoNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
