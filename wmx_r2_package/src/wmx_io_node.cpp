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

bool WmxIoNode::attachDevice()
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

  wmx3Lib_.SetDeviceName("wmx_io_node");
  isDeviceAttached_ = true;
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");
  return true;
}

void WmxIoNode::releaseDevice()
{
  if (!isDeviceAttached_) {
    return;
  }

  wmxIo_.reset();
  err_ = wmx3Lib_.CloseDevice();
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device");
  } else {
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }
  isDeviceAttached_ = false;
}

std::string WmxIoNode::notActiveMessage() const
{
  return "wmx_io_node is not active (state: " +
         this->get_current_state().label() + ").";
}

WmxIoNode::CallbackReturn WmxIoNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_io_node...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  wmxIo_ = std::make_unique<IO>(&wmx3Lib_);

  getInputBitService_ = this->create_service<wmx_r2_message::srv::GetIoBit>(
    "wmx/io/get_input_bit",
    std::bind(&WmxIoNode::getInputBit, this, _1, _2));

  getOutputBitService_ = this->create_service<wmx_r2_message::srv::GetIoBit>(
    "wmx/io/get_output_bit",
    std::bind(&WmxIoNode::getOutputBit, this, _1, _2));

  getInputBytesService_ = this->create_service<wmx_r2_message::srv::GetIoBytes>(
    "wmx/io/get_input_bytes",
    std::bind(&WmxIoNode::getInputBytes, this, _1, _2));

  getOutputBytesService_ = this->create_service<wmx_r2_message::srv::GetIoBytes>(
    "wmx/io/get_output_bytes",
    std::bind(&WmxIoNode::getOutputBytes, this, _1, _2));

  setOutputBitService_ = this->create_service<wmx_r2_message::srv::SetIoBit>(
    "wmx/io/set_output_bit",
    std::bind(&WmxIoNode::setOutputBit, this, _1, _2));

  setOutputBytesService_ = this->create_service<wmx_r2_message::srv::SetIoBytes>(
    "wmx/io/set_output_bytes",
    std::bind(&WmxIoNode::setOutputBytes, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_io_node is configured");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isActive_ = true;
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is active");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  isActive_ = false;
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  isActive_ = false;

  getInputBitService_.reset();
  getOutputBitService_.reset();
  getInputBytesService_.reset();
  getOutputBytesService_.reset();
  setOutputBitService_.reset();
  setOutputBytesService_.reset();

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_io_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void WmxIoNode::getInputBit(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  unsigned char data = 0;
  err_ = wmxIo_->GetInBitEx(request->byte, request->bit, &data);
  if (err_ != ErrorCode::None) {
    IO::ErrorToString(err_, errString_, sizeof(errString_));
    snprintf(
      buffer_, sizeof(buffer_),
      "GetInBitEx failed byte=%d bit=%d. Error=%d (%s)",
      request->byte, request->bit, err_, errString_);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = buffer_;
    return;
  }

  response->success = true;
  response->value = static_cast<int32_t>(data);
  snprintf(
    buffer_, sizeof(buffer_),
    "Input bit %d.%d = %d", request->byte, request->bit, data);
  response->message = buffer_;
}

void WmxIoNode::getOutputBit(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  unsigned char data = 0;
  err_ = wmxIo_->GetOutBitEx(request->byte, request->bit, &data);
  if (err_ != ErrorCode::None) {
    IO::ErrorToString(err_, errString_, sizeof(errString_));
    snprintf(
      buffer_, sizeof(buffer_),
      "GetOutBitEx failed byte=%d bit=%d. Error=%d (%s)",
      request->byte, request->bit, err_, errString_);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = buffer_;
    return;
  }

  response->success = true;
  response->value = static_cast<int32_t>(data);
  snprintf(
    buffer_, sizeof(buffer_),
    "Output bit %d.%d = %d", request->byte, request->bit, data);
  response->message = buffer_;
}

void WmxIoNode::getInputBytes(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  if (request->length <= 0) {
    response->success = false;
    response->message = "Invalid length: must be > 0";
    return;
  }

  std::vector<unsigned char> data(request->length, 0);
  err_ = wmxIo_->GetInBytesEx(request->byte, request->length, data.data());
  if (err_ != ErrorCode::None) {
    IO::ErrorToString(err_, errString_, sizeof(errString_));
    snprintf(
      buffer_, sizeof(buffer_),
      "GetInBytesEx failed byte=%d length=%d. Error=%d (%s)",
      request->byte, request->length, err_, errString_);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = buffer_;
    return;
  }

  response->success = true;
  response->data.assign(data.begin(), data.end());
  snprintf(
    buffer_, sizeof(buffer_),
    "Read %d input bytes from byte %d", request->length, request->byte);
  response->message = buffer_;
}

void WmxIoNode::getOutputBytes(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  if (request->length <= 0) {
    response->success = false;
    response->message = "Invalid length: must be > 0";
    return;
  }

  std::vector<unsigned char> data(request->length, 0);
  err_ = wmxIo_->GetOutBytesEx(request->byte, request->length, data.data());
  if (err_ != ErrorCode::None) {
    IO::ErrorToString(err_, errString_, sizeof(errString_));
    snprintf(
      buffer_, sizeof(buffer_),
      "GetOutBytesEx failed byte=%d length=%d. Error=%d (%s)",
      request->byte, request->length, err_, errString_);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = buffer_;
    return;
  }

  response->success = true;
  response->data.assign(data.begin(), data.end());
  snprintf(
    buffer_, sizeof(buffer_),
    "Read %d output bytes from byte %d", request->length, request->byte);
  response->message = buffer_;
}

void WmxIoNode::setOutputBit(
  const std::shared_ptr<wmx_r2_message::srv::SetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetIoBit::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  if (request->value != 0 && request->value != 1) {
    response->success = false;
    response->message = "Invalid value: must be 0 or 1";
    return;
  }

  err_ = wmxIo_->SetOutBitEx(request->byte, request->bit, (request->value ? 1 : 0));
  if (err_ != ErrorCode::None) {
    IO::ErrorToString(err_, errString_, sizeof(errString_));
    snprintf(
      buffer_, sizeof(buffer_),
      "SetOutBitEx failed byte=%d bit=%d value=%d. Error=%d (%s)",
      request->byte, request->bit, request->value, err_, errString_);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = buffer_;
    return;
  }

  snprintf(
    buffer_, sizeof(buffer_),
    "Set output bit %d.%d = %d", request->byte, request->bit, request->value);
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
  response->success = true;
  response->message = buffer_;
}

void WmxIoNode::setOutputBytes(
  const std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  if (request->data.empty()) {
    response->success = false;
    response->message = "No data provided";
    return;
  }

  int dataLen = static_cast<int>(request->data.size());
  std::vector<unsigned char> rawData(request->data.begin(), request->data.end());

  err_ = wmxIo_->SetOutBytesEx(request->byte, dataLen, rawData.data());
  if (err_ != ErrorCode::None) {
    IO::ErrorToString(err_, errString_, sizeof(errString_));
    snprintf(
      buffer_, sizeof(buffer_),
      "SetOutBytesEx failed byte=%d length=%d. Error=%d (%s)",
      request->byte, dataLen, err_, errString_);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    response->success = false;
    response->message = buffer_;
    return;
  }

  snprintf(
    buffer_, sizeof(buffer_),
    "Set %d output bytes from byte %d", dataLen, request->byte);
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
  response->success = true;
  response->message = buffer_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxIoNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
