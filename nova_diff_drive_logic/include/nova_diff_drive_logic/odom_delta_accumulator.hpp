// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
//
// Accumulates absolute distance/angle travelled between reads, for the Nova
// `/odom_deltas` topic. Ported from differential_drive_controller's
// update_deltas + send_odom_delta (accumulate |v|*dt, |w|*dt; reset on publish).
#ifndef NOVA_DIFF_DRIVE_LOGIC__ODOM_DELTA_ACCUMULATOR_HPP_
#define NOVA_DIFF_DRIVE_LOGIC__ODOM_DELTA_ACCUMULATOR_HPP_

#include <cmath>

#include "nova_diff_drive_logic/diff_drive_kinematics.hpp"

namespace nova_diff_drive_logic
{

/// Accumulated absolute travel since the last take(): linear [m], angular [rad].
struct OdomDelta
{
  double linear = 0.0;
  double angular = 0.0;
};

/// Accumulates |linear|*dt and |angular|*dt. take() returns the accumulated
/// values and resets to zero (publish-and-reset pattern).
class OdomDeltaAccumulator
{
public:
  void accumulate(const BodyVel & vel, double dt)
  {
    if (!std::isfinite(dt) || dt <= 0.0) {return;}  // ignore invalid/negative dt
    delta_.linear += std::abs(vel.linear * dt);
    delta_.angular += std::abs(vel.angular * dt);
  }

  /// Return accumulated deltas and reset the accumulator to zero.
  OdomDelta take()
  {
    const OdomDelta out = delta_;
    delta_ = {};
    return out;
  }

  const OdomDelta & peek() const {return delta_;}

private:
  OdomDelta delta_;
};

}  // namespace nova_diff_drive_logic

#endif  // NOVA_DIFF_DRIVE_LOGIC__ODOM_DELTA_ACCUMULATOR_HPP_
