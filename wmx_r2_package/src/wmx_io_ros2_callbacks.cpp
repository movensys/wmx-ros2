// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_io_node.hpp"

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

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
