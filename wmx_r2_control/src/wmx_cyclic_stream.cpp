// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_r2_control/wmx_cyclic_stream.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace wmx_r2_control
{

using wmx3Api::AxisCommandMode;
using wmx3Api::CyclicBufferCommandType;
using wmx3Api::CyclicBufferState;
using wmx3Api::ErrorCode;

WmxCyclicStream::~WmxCyclicStream()
{
  close();
}

void WmxCyclicStream::setError(std::string * error, const char * what, int code) const
{
  if (error == nullptr) {
    return;
  }
  char buf[256] = {0};
  wmx3Api::CyclicBuffer::ErrorToString(code, buf, sizeof(buf));
  *error = std::string(what) + ": error=" + std::to_string(code) + " (" + buf + ")";
}

bool WmxCyclicStream::open(
  wmx3Api::WMX3Api * wmx, wmx3Api::CoreMotion * cm, const Config & cfg,
  double seed_push_period_ms, std::string * error)
{
  if (open_) {
    if (error != nullptr) {*error = "cyclic stream is already open";}
    return false;
  }
  // Config first: it is the pure check, needing neither handles nor an engine,
  // and a bad axis list is the likelier real-world mistake (it comes from the
  // ros2_control xacro) so it deserves the more specific diagnostic.
  if (cfg.axes.empty() || cfg.axes.size() > static_cast<size_t>(wmx3Api::constants::maxAxes)) {
    if (error != nullptr) {
      *error = "axis list must hold between 1 and " +
        std::to_string(wmx3Api::constants::maxAxes) + " axes";
    }
    return false;
  }
  for (const int axis : cfg.axes) {
    if (axis < 0 || axis >= wmx3Api::constants::maxAxes) {
      if (error != nullptr) {*error = "axis " + std::to_string(axis) + " is out of range";}
      return false;
    }
  }
  if (wmx == nullptr || cm == nullptr) {
    if (error != nullptr) {*error = "null WMX3Api or CoreMotion handle";}
    return false;
  }

  wmx_ = wmx;
  cm_ = cm;
  cfg_ = cfg;

  sel_.axisCount = static_cast<int>(cfg_.axes.size());
  for (size_t i = 0; i < cfg_.axes.size(); ++i) {
    sel_.axis[i] = cfg_.axes[i];
  }

  // AbsolutePos commands require Position command mode; anything else aborts
  // execution into the latching CommandModeMismatchError state at the first
  // command. Check it here so a misconfigured axis fails at activation with a
  // clear message instead of going limp mid-motion.
  for (const int axis : cfg_.axes) {
    AxisCommandMode::T mode = AxisCommandMode::T::Position;
    const int err = cm_->axisControl->GetAxisCommandMode(axis, &mode);
    if (err != ErrorCode::None) {
      setError(error, "GetAxisCommandMode failed", err);
      return false;
    }
    if (mode != AxisCommandMode::T::Position) {
      if (error != nullptr) {
        *error = "axis " + std::to_string(axis) + " is not in Position command mode (mode=" +
          std::to_string(static_cast<int>(mode)) + "); AbsolutePos cyclic commands require it";
      }
      return false;
    }
  }

  // The engine communication cycle is not configured anywhere in this workspace
  // -- it belongs to the WMX engine. Read it; a hardcoded 0.5 ms would silently
  // scale every commanded velocity on a differently configured engine.
  wmx3Api::CoreMotionStatus status;
  const int err = cm_->GetStatus(&status);
  if (err != ErrorCode::None) {
    setError(error, "CoreMotion GetStatus failed", err);
    return false;
  }
  timing_ = cyclic::StreamTiming();
  timing_.cycle_ms = status.cycleTimeMilliseconds[0];
  timing_.push_period_ms = seed_push_period_ms;
  if (!timing_.valid()) {
    if (error != nullptr) {
      *error = "invalid stream timing (engine cycle=" + std::to_string(timing_.cycle_ms) +
        " ms, push period=" + std::to_string(timing_.push_period_ms) +
        " ms); is the engine communicating?";
    }
    return false;
  }
  period_ = std::make_unique<cyclic::PeriodTracker>(seed_push_period_ms);
  target_depth_ = timing_.targetDepth(cfg_.target_periods);

  cb_ = std::make_unique<wmx3Api::CyclicBuffer>(wmx_);

  // Allocates on the engine: activation only, never from the control loop.
  const unsigned int slots = timing_.bufferCommands(cfg_.buffer_horizon_s);
  const int open_err = cb_->OpenCyclicBuffer(&sel_, slots);
  if (open_err != ErrorCode::None) {
    setError(error, "OpenCyclicBuffer failed", open_err);
    cb_.reset();
    return false;
  }

  open_ = true;
  return true;
}

bool WmxCyclicStream::holdCommands(
  wmx3Api::CyclicBufferMultiAxisCommands * cmds, unsigned int interval, std::string * error)
{
  for (const int axis : cfg_.axes) {
    // The *command* position, not the feedback position. Seeding from feedback
    // (CoreMotionStatus::actualPos, which read() correctly uses for
    // /joint_states) would command a step equal to the servo following error on
    // the first cycle, simultaneously on every axis.
    double commanded = 0.0;
    const int err = cm_->axisControl->GetPosCommand(axis, &commanded);
    if (err != ErrorCode::None) {
      setError(error, "GetPosCommand failed", err);
      return false;
    }
    // Indexed by axis number, not by position in the selection: cmd[], option[]
    // and status[] are all maxAxes-wide arrays keyed on the axis.
    cmds->cmd[axis].type = CyclicBufferCommandType::T::AbsolutePos;
    cmds->cmd[axis].command = commanded;
    cmds->cmd[axis].intervalCycles = interval;
  }
  return true;
}

bool WmxCyclicStream::start(std::string * error)
{
  if (!open_) {
    if (error != nullptr) {*error = "cyclic stream is not open";}
    return false;
  }
  if (running_) {
    return true;
  }

  const unsigned int interval = timing_.nominalInterval();

  // Prime the setpoint's worth of runway holding the current pose, so the first
  // real command lands into a buffer that is already at depth rather than
  // starting from an underrun.
  for (unsigned int i = 0; i < target_depth_; ++i) {
    wmx3Api::CyclicBufferMultiAxisCommands hold;
    if (!holdCommands(&hold, interval, error)) {
      return false;
    }
    const int err = cb_->AddCommand(&sel_, &hold);
    if (err != ErrorCode::None) {
      setError(error, "AddCommand failed while priming the buffer", err);
      return false;
    }
  }

  wmx3Api::CyclicBufferMultiAxisOption option;
  for (const int axis : cfg_.axes) {
    option.option[axis].maxAcc = cfg_.max_acc;
  }

  // Execute is called once. When the buffer drains, the state becomes
  // WaitingForCommand and execution resumes automatically on the next
  // AddCommand -- there is no need (and no way) to re-Execute per command.
  const int err = cb_->Execute(&sel_, &option);
  if (err != ErrorCode::None) {
    setError(error, "Execute failed", err);
    return false;
  }

  running_ = true;
  return refresh(error);
}

bool WmxCyclicStream::queryBuffer(
  unsigned int * min_free, unsigned int * min_depth, std::string * error)
{
  wmx3Api::CyclicBufferMultiAxisStatus status;
  const int err = cb_->GetStatus(&sel_, &status);
  if (err != ErrorCode::None) {
    setError(error, "CyclicBuffer GetStatus failed", err);
    return false;
  }

  unsigned int free_slots = std::numeric_limits<unsigned int>::max();
  unsigned int depth = std::numeric_limits<unsigned int>::max();
  for (const int axis : cfg_.axes) {
    const auto & axis_status = status.status[axis];
    // availableCount is the buffer's capacity, not its free space, despite what
    // its API doc comment suggests; the guide's formula is
    // free = availableCount - remainCount.
    const unsigned int axis_free = (axis_status.availableCount > axis_status.remainCount) ?
      (axis_status.availableCount - axis_status.remainCount) : 0u;
    free_slots = std::min(free_slots, axis_free);
    depth = std::min(depth, axis_status.remainCount);
  }

  // Report the first axis' state: they are commanded together, so they share a
  // fate, and a per-axis breakdown belongs in the caller's diagnostics.
  state_ = status.status[cfg_.axes.front()].state;
  *min_free = free_slots;
  *min_depth = depth;
  return true;
}

bool WmxCyclicStream::refresh(std::string * error)
{
  if (!open_) {
    if (error != nullptr) {*error = "cyclic stream is not open";}
    return false;
  }
  unsigned int free_slots = 0u;
  unsigned int depth = 0u;
  if (!queryBuffer(&free_slots, &depth, error)) {
    return false;
  }
  depth_ = depth;
  return true;
}

bool WmxCyclicStream::push(const std::vector<double> & positions, std::string * error)
{
  if (!running_) {
    if (error != nullptr) {*error = "cyclic stream is not running";}
    return false;
  }
  if (positions.size() != cfg_.axes.size()) {
    if (error != nullptr) {
      *error = "expected " + std::to_string(cfg_.axes.size()) + " positions, got " +
        std::to_string(positions.size());
    }
    return false;
  }
  for (const double q : positions) {
    if (!std::isfinite(q)) {
      ++rejected_;
      if (error != nullptr) {*error = "refusing to command a non-finite position";}
      return false;
    }
  }

  unsigned int free_slots = 0u;
  unsigned int depth = 0u;
  if (!queryBuffer(&free_slots, &depth, error)) {
    return false;
  }
  depth_ = depth;

  if (faulted()) {
    ++rejected_;
    if (error != nullptr) {
      *error = std::string("cyclic buffer is in a latching error state (") + stateName() +
        "); abort and restart the stream to recover";
    }
    return false;
  }

  // Check every axis has room before committing to any of them. The multi-axis
  // AddCommand applies *partially* on BufferSizeIsNotEnough -- axes ordered
  // before the failing one keep their command -- which on a 6-DOF arm desyncs
  // the joints rather than merely dropping a frame.
  if (free_slots < 1u) {
    ++rejected_;
    if (error != nullptr) {
      *error = "cyclic buffer full (depth=" + std::to_string(depth_) + "); dropping command";
    }
    return false;
  }

  const unsigned int interval = cyclic::nextInterval(timing_, target_depth_, depth_);

  wmx3Api::CyclicBufferMultiAxisCommands cmds;
  for (size_t i = 0; i < cfg_.axes.size(); ++i) {
    const int axis = cfg_.axes[i];
    cmds.cmd[axis].type = CyclicBufferCommandType::T::AbsolutePos;
    cmds.cmd[axis].command = positions[i];
    cmds.cmd[axis].intervalCycles = interval;
  }

  const int err = cb_->AddCommand(&sel_, &cmds);
  if (err != ErrorCode::None) {
    ++rejected_;
    setError(error, "AddCommand failed", err);
    return false;
  }
  return true;
}

void WmxCyclicStream::observePushPeriod(double measured_ms)
{
  if (period_ == nullptr) {
    return;
  }
  period_->observe(measured_ms);
  timing_.push_period_ms = period_->periodMs();
  target_depth_ = timing_.targetDepth(cfg_.target_periods);
}

void WmxCyclicStream::abort()
{
  if (!open_ || cb_ == nullptr) {
    running_ = false;
    return;
  }
  // Abort, not ExecQuickStop: Abort stops immediately and can be re-armed on the
  // very next cycle, whereas ExecQuickStop blocks AddCommand and Execute until
  // the state returns to Stopped, silently swallowing the first commands of the
  // next engage.
  cb_->Abort(&sel_);
  running_ = false;
  depth_ = 0u;
  state_ = CyclicBufferState::T::Stopped;
}

void WmxCyclicStream::close()
{
  if (!open_) {
    return;
  }
  abort();
  if (cb_ != nullptr) {
    cb_->CloseCyclicBuffer(&sel_);
    cb_.reset();
  }
  period_.reset();
  open_ = false;
}

bool WmxCyclicStream::faulted() const
{
  switch (state_) {
    // Both clear the buffer and latch: another push cannot recover them.
    case CyclicBufferState::T::MaxAccError:
    case CyclicBufferState::T::CommandModeMismatchError:
    // Not in the vendored markdown docs but present in CyclicBufferApi.h. A
    // dropped servo must be treated as a fault, not as a transient, or the
    // stream would keep pushing into an axis that cannot move.
    case CyclicBufferState::T::ServoOff:
      return true;
    default:
      return false;
  }
}

const char * WmxCyclicStream::stateName() const
{
  switch (state_) {
    case CyclicBufferState::T::Stopped:
      return "Stopped";
    case CyclicBufferState::T::ExecutingCommand:
      return "ExecutingCommand";
    case CyclicBufferState::T::WaitingForCommand:
      return "WaitingForCommand";
    case CyclicBufferState::T::MaxAccError:
      return "MaxAccError";
    case CyclicBufferState::T::CommandModeMismatchError:
      return "CommandModeMismatchError";
    case CyclicBufferState::T::Busy:
      return "Busy";
    case CyclicBufferState::T::ServoOff:
      return "ServoOff";
    default:
      return "Unknown";
  }
}

}  // namespace wmx_r2_control
