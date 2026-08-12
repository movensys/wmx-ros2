// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_engine_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

WmxEngineNode::WmxEngineNode()
: Node("wmx_engine_node")
{
  devicePath_ = this->declare_parameter<std::string>("device_path", "/opt/wmx3/");
  deviceName_ = this->declare_parameter<std::string>("device_name", "wmx_r2");
  engineCore_ = this->declare_parameter<int>("engine_core", -1);
  engineAffinityMask_ = this->declare_parameter<int64_t>("engine_affinity_mask", 0);
  wmxParamFilePath_ = this->declare_parameter<std::string>("wmx_param_file_path", "");
  managedNodes_ = this->declare_parameter<std::vector<std::string>>(
    "managed_nodes", std::vector<std::string>{});

  managerCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lifecycleClientCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  setEngineService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_engine",
    std::bind(&WmxEngineNode::setEngineCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  setCommService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_comm",
    std::bind(&WmxEngineNode::setCommCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/engine/get_status",
    std::bind(&WmxEngineNode::getEngineStatusCallback, this, _1, _2));

  loadParamsService_ = this->create_service<wmx_r2_message::srv::LoadWmxParams>(
    "wmx/params/load",
    std::bind(&WmxEngineNode::loadWmxParamsCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  getParamsService_ = this->create_service<wmx_r2_message::srv::GetWmxParams>(
    "wmx/params/get",
    std::bind(&WmxEngineNode::getWmxParamsCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  setNodeStateService_ = this->create_service<wmx_r2_message::srv::SetNodeState>(
    "wmx/engine/set_node_state",
    std::bind(&WmxEngineNode::setNodeStateCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  getNodeStatesService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/engine/get_node_states",
    std::bind(&WmxEngineNode::getNodeStatesCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  discoveryTimer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&WmxEngineNode::discoveryStep, this),
    managerCbGroup_);

  startThread_ = std::thread(&WmxEngineNode::wmxStartEngine, this);

  RCLCPP_INFO(this->get_logger(), "wmx_engine_node is ready");
}

WmxEngineNode::~WmxEngineNode()
{
  if (startThread_.joinable()) {
    startThread_.join();
  }
  wmxStopCommunication();
  wmxStopEngine();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(this->get_logger(), "wmx_engine_node stopped");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxEngineNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
