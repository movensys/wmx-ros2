// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_io_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

namespace
{
std::string errorToString(int err)
{
  char errString[256] = {};
  IO::ErrorToString(err, errString, sizeof(errString));
  return errString;
}

std::string failureText(const std::string & call, const std::string & where, int err)
{
  return call + " failed " + where + ". Error=" + std::to_string(err) +
         " (" + errorToString(err) + ")";
}
}  // namespace

WmxIoNodeApi::WmxIoNodeApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

WmxIoNodeApi::~WmxIoNodeApi()
{
  closeDevice();
}

int WmxIoNodeApi::createDevice(std::string & message)
{
  if (wmxIo_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    if (err == ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device (lock busy). Is the engine started?";
    } else {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errorToString(err) + ")";
    }
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = wmx3Lib_.SetDeviceName(deviceName_);
  if (err != ErrorCode::None) {
    message = "Failed to name the device '" + std::string(deviceName_) + "'. Error=" +
      std::to_string(err) + " (" + errorToString(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    wmx3Lib_.CloseDevice();
    return err;
  }

  wmxIo_ = std::make_unique<IO>(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxIoNodeApi::closeDevice()
{
  wmxIo_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorToString(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int WmxIoNodeApi::getInBit(int32_t addr, int32_t bit, uint8_t & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read input bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = wmxIo_->GetInBitEx(addr, bit, &data);
  if (err != ErrorCode::None) {
    message = failureText(
      "GetInBitEx", "addr=" + std::to_string(addr) + " bit=" + std::to_string(bit), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Input bit " + std::to_string(addr) + "." + std::to_string(bit) + " = " +
    std::to_string(data);
  return ErrorCode::None;
}

int WmxIoNodeApi::getOutBit(int32_t addr, int32_t bit, uint8_t & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read output bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = wmxIo_->GetOutBitEx(addr, bit, &data);
  if (err != ErrorCode::None) {
    message = failureText(
      "GetOutBitEx", "addr=" + std::to_string(addr) + " bit=" + std::to_string(bit), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Output bit " + std::to_string(addr) + "." + std::to_string(bit) + " = " +
    std::to_string(data);
  return ErrorCode::None;
}

int WmxIoNodeApi::getInBytes(
  int32_t addr, int32_t size, std::vector<uint8_t> & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read input bytes. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (size <= 0 || size > wmx3Api::constants::maxIOInSize) {
    message = "Invalid size: must be 1.." +
      std::to_string(wmx3Api::constants::maxIOInSize);
    return ErrorCode::IOSizeOutOfRange;
  }

  std::vector<unsigned char> raw(size, 0);
  const int err = wmxIo_->GetInBytesEx(addr, size, raw.data());
  if (err != ErrorCode::None) {
    message = failureText(
      "GetInBytesEx", "addr=" + std::to_string(addr) + " size=" + std::to_string(size), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  data.assign(raw.begin(), raw.end());
  message = "Read " + std::to_string(size) + " input bytes from addr " + std::to_string(addr);
  return ErrorCode::None;
}

int WmxIoNodeApi::getOutBytes(
  int32_t addr, int32_t size, std::vector<uint8_t> & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read output bytes. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (size <= 0 || size > wmx3Api::constants::maxIOOutSize) {
    message = "Invalid size: must be 1.." +
      std::to_string(wmx3Api::constants::maxIOOutSize);
    return ErrorCode::IOSizeOutOfRange;
  }

  std::vector<unsigned char> raw(size, 0);
  const int err = wmxIo_->GetOutBytesEx(addr, size, raw.data());
  if (err != ErrorCode::None) {
    message = failureText(
      "GetOutBytesEx", "addr=" + std::to_string(addr) + " size=" + std::to_string(size), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  data.assign(raw.begin(), raw.end());
  message = "Read " + std::to_string(size) + " output bytes from addr " + std::to_string(addr);
  return ErrorCode::None;
}

int WmxIoNodeApi::setOutBit(int32_t addr, int32_t bit, uint8_t data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot set output bit. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (data != 0 && data != 1) {
    message = "Invalid data: must be 0 or 1";
    return ErrorCode::ArgumentOutOfRange;
  }

  const int err = wmxIo_->SetOutBitEx(addr, bit, (data ? 1 : 0));
  if (err != ErrorCode::None) {
    message = failureText(
      "SetOutBitEx",
      "addr=" + std::to_string(addr) + " bit=" + std::to_string(bit) +
      " data=" + std::to_string(data), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set output bit " + std::to_string(addr) + "." + std::to_string(bit) + " = " +
    std::to_string(data);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxIoNodeApi::getInByte(int32_t addr, uint8_t & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read input byte. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = wmxIo_->GetInByteEx(addr, &data);
  if (err != ErrorCode::None) {
    message = failureText("GetInByteEx", "addr=" + std::to_string(addr), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Input byte " + std::to_string(addr) + " = " + std::to_string(data);
  return ErrorCode::None;
}

int WmxIoNodeApi::getOutByte(int32_t addr, uint8_t & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot read output byte. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = wmxIo_->GetOutByteEx(addr, &data);
  if (err != ErrorCode::None) {
    message = failureText("GetOutByteEx", "addr=" + std::to_string(addr), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Output byte " + std::to_string(addr) + " = " + std::to_string(data);
  return ErrorCode::None;
}

int WmxIoNodeApi::setOutByte(int32_t addr, uint8_t data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot set output byte. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = wmxIo_->SetOutByteEx(addr, data);
  if (err != ErrorCode::None) {
    message = failureText(
      "SetOutByteEx",
      "addr=" + std::to_string(addr) + " data=" + std::to_string(data), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set output byte " + std::to_string(addr) + " = " + std::to_string(data);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxIoNodeApi::setOutBits(
  const std::vector<int32_t> & addr, const std::vector<int32_t> & bit,
  const std::vector<uint8_t> & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot set output bits. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (addr.empty()) {
    message = "No data provided";
    return ErrorCode::IOSizeOutOfRange;
  }

  if (bit.size() != addr.size() || data.size() != addr.size()) {
    message = "addr, bit and data must be the same size";
    return ErrorCode::ArgumentOutOfRange;
  }

  for (const uint8_t bitData : data) {
    if (bitData != 0 && bitData != 1) {
      message = "Invalid data: must be 0 or 1";
      return ErrorCode::ArgumentOutOfRange;
    }
  }

  const int count = static_cast<int>(addr.size());
  std::vector<int> rawAddr(addr.begin(), addr.end());
  std::vector<int> rawBit(bit.begin(), bit.end());
  std::vector<unsigned char> rawData(data.begin(), data.end());

  const int err = wmxIo_->SetOutBitsEx(
    rawAddr.data(), rawBit.data(), rawData.data(), count);
  if (err != ErrorCode::None) {
    message = failureText("SetOutBitsEx", "count=" + std::to_string(count), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set " + std::to_string(count) + " output bits";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxIoNodeApi::setOutBytes(
  int32_t addr, const std::vector<uint8_t> & data, std::string & message)
{
  if (!wmxIo_) {
    message = "Cannot set output bytes. IO is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (data.empty()) {
    message = "No data provided";
    return ErrorCode::IOSizeOutOfRange;
  }

  const int size = static_cast<int>(data.size());
  std::vector<unsigned char> raw(data.begin(), data.end());

  const int err = wmxIo_->SetOutBytesEx(addr, size, raw.data());
  if (err != ErrorCode::None) {
    message = failureText(
      "SetOutBytesEx", "addr=" + std::to_string(addr) + " size=" + std::to_string(size), err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set " + std::to_string(size) + " output bytes from addr " + std::to_string(addr);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

WmxIoNode::WmxIoNode()
: LifecycleNode("wmx_io_node")
{
  api_ = std::make_unique<WmxIoNodeApi>(this->get_logger());
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is unconfigured, waiting for configure...");
}

WmxIoNode::~WmxIoNode()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "wmx_io_node stopped");
}

bool WmxIoNode::isNodeActive()
{
  return this->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

std::string WmxIoNode::notActiveMessage()
{
  return "wmx_io_node is not active (state: " +
         this->get_current_state().label() + ").";
}

WmxIoNode::CallbackReturn WmxIoNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_io_node...");

  std::string message;
  if (api_->createDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  getInBitService_ = this->create_service<wmx_r2_message::srv::GetIoBit>(
    "wmx/io/get_in_bit",
    std::bind(&WmxIoNode::getInBitCallback, this, _1, _2));

  getOutBitService_ = this->create_service<wmx_r2_message::srv::GetIoBit>(
    "wmx/io/get_out_bit",
    std::bind(&WmxIoNode::getOutBitCallback, this, _1, _2));

  getInBytesService_ = this->create_service<wmx_r2_message::srv::GetIoBytes>(
    "wmx/io/get_in_bytes",
    std::bind(&WmxIoNode::getInBytesCallback, this, _1, _2));

  getOutBytesService_ = this->create_service<wmx_r2_message::srv::GetIoBytes>(
    "wmx/io/get_out_bytes",
    std::bind(&WmxIoNode::getOutBytesCallback, this, _1, _2));

  setOutBitService_ = this->create_service<wmx_r2_message::srv::SetIoBit>(
    "wmx/io/set_out_bit",
    std::bind(&WmxIoNode::setOutBitCallback, this, _1, _2));

  getInByteService_ = this->create_service<wmx_r2_message::srv::GetIoByte>(
    "wmx/io/get_in_byte",
    std::bind(&WmxIoNode::getInByteCallback, this, _1, _2));

  getOutByteService_ = this->create_service<wmx_r2_message::srv::GetIoByte>(
    "wmx/io/get_out_byte",
    std::bind(&WmxIoNode::getOutByteCallback, this, _1, _2));

  setOutBitsService_ = this->create_service<wmx_r2_message::srv::SetIoBits>(
    "wmx/io/set_out_bits",
    std::bind(&WmxIoNode::setOutBitsCallback, this, _1, _2));

  setOutByteService_ = this->create_service<wmx_r2_message::srv::SetIoByte>(
    "wmx/io/set_out_byte",
    std::bind(&WmxIoNode::setOutByteCallback, this, _1, _2));

  setOutBytesService_ = this->create_service<wmx_r2_message::srv::SetIoBytes>(
    "wmx/io/set_out_bytes",
    std::bind(&WmxIoNode::setOutBytesCallback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_io_node is configured");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is active");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  getInBitService_.reset();
  getOutBitService_.reset();
  getInBytesService_.reset();
  getOutBytesService_.reset();
  setOutBitService_.reset();
  getInByteService_.reset();
  getOutByteService_.reset();
  setOutBitsService_.reset();
  setOutByteService_.reset();
  setOutBytesService_.reset();

  api_->closeDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_io_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void WmxIoNode::getInBitCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->getInBit(request->addr, request->bit, response->data, message) == ErrorCode::None;
  response->message = message;
}

void WmxIoNode::getOutBitCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->getOutBit(request->addr, request->bit, response->data, message) == ErrorCode::None;
  response->message = message;
}

void WmxIoNode::getInBytesCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->getInBytes(request->addr, request->size, response->data, message) ==
    ErrorCode::None;
  response->message = message;
}

void WmxIoNode::getOutBytesCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->getOutBytes(request->addr, request->size, response->data, message) ==
    ErrorCode::None;
  response->message = message;
}

void WmxIoNode::setOutBitCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetIoBit::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetIoBit::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->setOutBit(request->addr, request->bit, request->data, message) == ErrorCode::None;
  response->message = message;
}

void WmxIoNode::setOutBytesCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->setOutBytes(request->addr, request->data, message) == ErrorCode::None;
  response->message = message;
}

void WmxIoNode::getInByteCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoByte::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoByte::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->getInByte(request->addr, response->data, message) == ErrorCode::None;
  response->message = message;
}

void WmxIoNode::getOutByteCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetIoByte::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetIoByte::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->getOutByte(request->addr, response->data, message) == ErrorCode::None;
  response->message = message;
}

void WmxIoNode::setOutBitsCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetIoBits::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetIoBits::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->setOutBits(request->addr, request->bit, request->data, message) ==
    ErrorCode::None;
  response->message = message;
}

void WmxIoNode::setOutByteCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetIoByte::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetIoByte::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  std::string message;
  response->success =
    api_->setOutByte(request->addr, request->data, message) == ErrorCode::None;
  response->message = message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxIoNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
