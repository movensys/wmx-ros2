// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_ethercat_node.hpp"

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;

WmxEtherCatNode::WmxEtherCatNode()
: LifecycleNode("wmx_ethercat_node"), wmxEcat_(&wmx3Lib_)
{
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is unconfigured, waiting for configure...");
}

WmxEtherCatNode::~WmxEtherCatNode()
{
  releaseDevice();
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node stopped");
}

std::string WmxEtherCatNode::notActiveMessage() const
{
  return "wmx_ethercat_node is not active (state: " +
         this->get_current_state().label() + ").";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxEtherCatNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
