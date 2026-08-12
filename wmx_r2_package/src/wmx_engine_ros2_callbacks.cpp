// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_engine_node.hpp"

void WmxEngineNode::getEngineStatusCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!isEngineStarted_) {
    response->success = false;
    response->message = "Engine is not started yet";
    return;
  }

  wmx3Lib_.GetEngineStatus(&engineStatus_);

  switch (engineStatus_.state) {
    case wmx3Api::EngineState::Idle:          statusStr_ = "Idle"; break;
    case wmx3Api::EngineState::Running:       statusStr_ = "Running"; break;
    case wmx3Api::EngineState::Communicating: statusStr_ = "Communicating"; break;
    case wmx3Api::EngineState::Shutdown:      statusStr_ = "Shutdown"; break;
    case wmx3Api::EngineState::Unknown:       statusStr_ = "Unknown"; break;
    default:                                  statusStr_ = "Invalid"; break;
  }

  response->success = true;
  response->message = statusStr_;
}

void WmxEngineNode::setCommCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!isEngineStarted_) {
    response->success = false;
    response->message = "Engine is not started yet";
    return;
  }

  if (request->data) {
    err_ = wmxStartCommunication();
    if (err_ == wmx3Api::ErrorCode::None) {
      discoveryStep();
    }
  } else {
    bringDownDiscoveredNodes(false);
    err_ = wmxStopCommunication();
  }

  if (err_ == wmx3Api::ErrorCode::None) {
    snprintf(
      buffer_, sizeof(buffer_), "Communication %s",
      request->data ? "started" : "stopped");
  } else {
    snprintf(
      buffer_, sizeof(buffer_), "Failed to %s communication. Error=%d",
      request->data ? "start" : "stop", err_);
  }

  response->success = (err_ == wmx3Api::ErrorCode::None);
  response->message = buffer_;
}

void WmxEngineNode::setEngineCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (wmx3LibCm_) {
      response->success = false;
      response->message = "Engine is already started";
      return;
    }
    err_ = wmxStartEngine();
  } else {
    if (!wmx3LibCm_) {
      response->success = false;
      response->message = "Engine is not started";
      return;
    }
    bringDownDiscoveredNodes(true);
    err_ = wmxStopEngine();
  }

  if (err_ == wmx3Api::ErrorCode::None) {
    snprintf(
      buffer_, sizeof(buffer_), "Engine %s",
      request->data ? "started" : "stopped");
  } else {
    snprintf(
      buffer_, sizeof(buffer_), "Failed to %s engine. Error=%d",
      request->data ? "start" : "stop", err_);
  }

  response->success = (err_ == wmx3Api::ErrorCode::None);
  response->message = buffer_;
}

void WmxEngineNode::loadWmxParamsCallback(
  const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response)
{
  if (!isEngineStarted_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  response->success = wmxLoadParam(request->file_path);
  response->message = buffer_;
}

void WmxEngineNode::getWmxParamsCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response)
{
  if (!isEngineStarted_ || !wmx3LibCm_) {
    response->success = false;
    response->message = "No device: create it with wmx/engine/set_engine first.";
    return;
  }

  wmxGetParam(request->index, response->params_dump);

  response->success = true;
  response->message = "OK";
}
