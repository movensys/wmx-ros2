// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
//
// Accumulates absolute distance/angle travelled between reads, for the Nova
// `/odom_deltas` topic. Two modes (reset on publish via take()):
//   accumulate(vel, dt)       -- |v|*dt, |w|*dt   (velocity path; legacy, unit-tested)
//   accumulateDelta(ds, dtheta) -- |ds|, |dtheta| (encoder position-delta path; in use)
// The controller drives the position-delta path; both produce the same value semantics.
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

  /// Accumulate absolute body displacement (ds [m], dtheta [rad]) directly — for
  /// encoder-position-delta dead reckoning (dt-free; more exact than |v|*dt).
  void accumulateDelta(double ds, double dtheta)
  {
    if (!std::isfinite(ds) || !std::isfinite(dtheta)) {return;}  // ignore invalid steps
    delta_.linear += std::abs(ds);
    delta_.angular += std::abs(dtheta);
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
