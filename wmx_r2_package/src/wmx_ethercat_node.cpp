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

bool WmxEtherCatNode::attachDevice()
{
  if (isDeviceAttached_) {
    return true;
  }

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device (lock busy). Is the engine communicating?");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return false;
  }

  wmx3Lib_.SetDeviceName("wmx_ethercat_node");
  isDeviceAttached_ = true;
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");
  return true;
}

void WmxEtherCatNode::releaseDevice()
{
  if (!isDeviceAttached_) {
    return;
  }

  err_ = wmx3Lib_.CloseDevice();
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device");
  } else {
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }
  isDeviceAttached_ = false;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_ethercat_node...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  getNetworkStateService_ = this->create_service<wmx_r2_message::srv::EcatGetNetworkState>(
    "wmx/ecat/get_network_state",
    std::bind(&WmxEtherCatNode::getNetworkState, this, _1, _2));

  registerReadService_ = this->create_service<wmx_r2_message::srv::EcatRegisterRead>(
    "wmx/ecat/register_read",
    std::bind(&WmxEtherCatNode::registerRead, this, _1, _2));

  resetStatisticsService_ = this->create_service<wmx_r2_message::srv::EcatResetStatistics>(
    "wmx/ecat/reset_statistics",
    std::bind(&WmxEtherCatNode::resetStatistics, this, _1, _2));

  scanNetworkService_ = this->create_service<wmx_r2_message::srv::EcatScanNetwork>(
    "wmx/ecat/scan_network",
    std::bind(&WmxEtherCatNode::scanNetwork, this, _1, _2));

  startHotconnectService_ = this->create_service<wmx_r2_message::srv::EcatStartHotconnect>(
    "wmx/ecat/start_hotconnect",
    std::bind(&WmxEtherCatNode::startHotconnect, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is configured");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isActive_ = true;
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is active");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  isActive_ = false;
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  isActive_ = false;

  getNetworkStateService_.reset();
  registerReadService_.reset();
  resetStatisticsService_.reset();
  scanNetworkService_.reset();
  startHotconnectService_.reset();

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void WmxEtherCatNode::getNetworkState(
  const std::shared_ptr<wmx_r2_message::srv::EcatGetNetworkState::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatGetNetworkState::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  wmx3Api::ecApi::EcMasterInfo info;
  err_ = wmxEcat_.GetMasterInfo(request->master_id, &info);

  if (err_ != ErrorCode::None) {
    char ecErrString[256];
    wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
    snprintf(
      buffer_, sizeof(buffer_),
      "GetMasterInfo failed. masterId=%d Error=%d (%s)",
      request->master_id, err_, ecErrString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = std::string(buffer_);
    return;
  }

  // Master fields
  response->master_state = static_cast<int32_t>(info.state);
  response->master_mode = static_cast<int32_t>(info.mode);
  response->comm_period = info.statisticsInfo.commPeriod;
  response->total_axes = info.statisticsInfo.totalAxesNum;
  response->total_input_size = info.statisticsInfo.totalInputSize;
  response->total_output_size = info.statisticsInfo.totalOutputSize;
  response->ring_num = info.statisticsInfo.ringNum;
  response->total_rxpdo_size = info.statisticsInfo.totalRxPdoSize;
  response->total_txpdo_size = info.statisticsInfo.totalTxPdoSize;
  response->tx_delay = info.statisticsInfo.txDelay;
  response->min_tx_delay = info.statisticsInfo.minTxDelay;
  response->max_tx_delay = info.statisticsInfo.maxTxDelay;
  response->packet_loss = info.statisticsInfo.packetLoss;
  response->packet_timeout = info.statisticsInfo.packetTimeout;
  response->over_cycle = info.statisticsInfo.overCycle;
  response->num_of_slaves = static_cast<int32_t>(info.numOfSlaves);

  // Per-slave arrays
  for (unsigned int i = 0; i < info.numOfSlaves; ++i) {
    const wmx3Api::ecApi::EcSlaveInfo & s = info.slaves[i];
    response->slave_ids.push_back(static_cast<int32_t>(s.id));
    response->slave_states.push_back(static_cast<int32_t>(s.state));
    response->slave_al_codes.push_back(static_cast<int32_t>(s.alStatusCode));
    response->slave_positions.push_back(static_cast<int32_t>(s.position));
    response->slave_addresses.push_back(static_cast<int32_t>(s.address));
    response->slave_offline.push_back(s.offline);
    response->slave_inaccessible.push_back(s.inaccessible);
    response->slave_new.push_back(s.newSlave);
    response->slave_reverse.push_back(s.reverseSlave);
    response->slave_vendor_ids.push_back(s.vendorId);
    response->slave_product_codes.push_back(s.productCode);
    response->slave_revision_nos.push_back(s.revisionNo);
    response->slave_serial_nos.push_back(s.serialNo);
    response->slave_aliases.push_back(s.alias);
    response->slave_input_addrs.push_back(static_cast<int32_t>(s.inputAddr));
    response->slave_input_sizes.push_back(static_cast<int32_t>(s.inputSize));
    response->slave_output_addrs.push_back(static_cast<int32_t>(s.outputAddr));
    response->slave_output_sizes.push_back(static_cast<int32_t>(s.outputSize));
    response->slave_num_of_axes.push_back(static_cast<int32_t>(s.numOfAxes));
  }

  snprintf(
    buffer_, sizeof(buffer_),
    "Master %d: slaves=%u", request->master_id, info.numOfSlaves);
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
  response->success = true;
  response->message = std::string(buffer_);
}

void WmxEtherCatNode::registerRead(
  const std::shared_ptr<wmx_r2_message::srv::EcatRegisterRead::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatRegisterRead::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  if (request->reg_address < 0 || request->reg_address > 0xFFF) {
    response->success = false;
    response->message = "Invalid reg_address: must be in [0x000, 0xFFF].";
    return;
  }

  if (request->length <= 0 || request->length > 0x1000) {
    response->success = false;
    response->message = "Invalid length: must be in [1, 4096].";
    return;
  }

  if (request->reg_address + request->length > 0x1000) {
    response->success = false;
    response->message = "reg_address + length exceeds 0x1000.";
    return;
  }

  std::vector<unsigned char> buf(request->length, 0);

  err_ = wmxEcat_.RegisterRead(
    request->master_id,
    request->slave_id,
    request->reg_address,
    request->length,
    buf.data());

  if (err_ != ErrorCode::None) {
    char ecErrString[256];
    wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
    snprintf(
      buffer_, sizeof(buffer_),
      "RegisterRead failed. slaveId=%d reg=0x%03X length=%d Error=%d (%s)",
      request->slave_id, request->reg_address, request->length, err_, ecErrString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = std::string(buffer_);
    return;
  }

  response->data.assign(buf.begin(), buf.end());
  snprintf(
    buffer_, sizeof(buffer_),
    "RegisterRead success. slaveId=%d reg=0x%03X length=%d",
    request->slave_id, request->reg_address, request->length);
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
  response->success = true;
  response->message = std::string(buffer_);
}

void WmxEtherCatNode::resetStatistics(
  const std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  err_ = wmxEcat_.ResetRefClockInfo(request->master_id);
  if (err_ != ErrorCode::None) {
    char ecErrString[256];
    wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
    snprintf(
      buffer_, sizeof(buffer_),
      "ResetRefClockInfo failed. masterId=%d Error=%d (%s)",
      request->master_id, err_, ecErrString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = std::string(buffer_);
    return;
  }

  err_ = wmxEcat_.ResetTransmitStatisticsInfo(request->master_id);
  if (err_ != ErrorCode::None) {
    char ecErrString[256];
    wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
    snprintf(
      buffer_, sizeof(buffer_),
      "ResetTransmitStatisticsInfo failed. masterId=%d Error=%d (%s)",
      request->master_id, err_, ecErrString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = std::string(buffer_);
    return;
  }

  wmxEcat_.ScanNetwork(request->master_id);

  snprintf(
    buffer_, sizeof(buffer_),
    "Statistics reset and ScanNetwork done. masterId=%d", request->master_id);
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
  response->success = true;
  response->message = std::string(buffer_);
}

void WmxEtherCatNode::scanNetwork(
  const std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  err_ = wmxEcat_.ScanNetwork(request->master_id);

  if (err_ != ErrorCode::None) {
    char ecErrString[256];
    wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
    snprintf(
      buffer_, sizeof(buffer_),
      "ScanNetwork failed. masterId=%d Error=%d (%s)",
      request->master_id, err_, ecErrString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = std::string(buffer_);
    return;
  }

  snprintf(
    buffer_, sizeof(buffer_),
    "ScanNetwork done. masterId=%d", request->master_id);
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
  response->success = true;
  response->message = std::string(buffer_);
}

void WmxEtherCatNode::startHotconnect(
  const std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Request> request,
  std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  err_ = wmxEcat_.StartHotconnect(request->master_id);

  if (err_ != ErrorCode::None) {
    char ecErrString[256];
    wmx3Api::ecApi::Ecat::ErrorToString(err_, ecErrString, sizeof(ecErrString));
    snprintf(
      buffer_, sizeof(buffer_),
      "StartHotconnect failed. masterId=%d Error=%d (%s)",
      request->master_id, err_, ecErrString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = std::string(buffer_);
    return;
  }

  snprintf(
    buffer_, sizeof(buffer_),
    "StartHotconnect done. masterId=%d", request->master_id);
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
  response->success = true;
  response->message = std::string(buffer_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxEtherCatNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
