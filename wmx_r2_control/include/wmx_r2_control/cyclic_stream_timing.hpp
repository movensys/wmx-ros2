// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// Timing logic for streaming position commands into a WMX3 cyclic buffer from
// the ros2_control write() loop.
//
// WHY A CONTROLLER IS NEEDED AT ALL
//
// The cyclic buffer is a FIFO with no preemption: AddCommand *appends*, it does
// not replace what is already pending. So the motion duration pushed per call
// must, on average, equal the interval between calls. Push more and the queue
// grows without bound (commanded motion falls further and further behind the
// operator); push less and the queue drains empty, at which point the axis holds
// its last position and then steps when the next command lands -- a jerk on
// every underrun. At the nominal rate the two are exactly balanced, so loop
// jitter alone will walk the buffer one way or the other.
//
// intervalCycles is the handle: one command spread over N communication cycles,
// linearly interpolated by the engine, still occupying ONE buffer slot. Slightly
// stretching or compressing it steers the queue back toward a depth setpoint.
//
// UNITS: DEPTH IS MEASURED IN COMMANDS, NOT CYCLES
//
// CyclicBufferSingleAxisStatus::remainCount counts queued *commands* (slots),
// not cycles -- a command occupies one slot regardless of its intervalCycles.
// Since each pushed command carries about one push-period of motion, a depth of
// D commands is about D push-periods of runway. Depth is therefore a small
// integer and the correction below saturates at its clamp for any error of one
// command or more; see nextInterval() for what that means in practice.
//
// Kept free of WMX and ROS types so it can be unit-tested anywhere -- the WMX3
// SDK is not installed on CI runners. WmxCyclicStream is the thin wrapper that
// turns these numbers into CyclicBuffer API calls.
#ifndef WMX_R2_CONTROL__CYCLIC_STREAM_TIMING_HPP_
#define WMX_R2_CONTROL__CYCLIC_STREAM_TIMING_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace wmx_r2_control
{
namespace cyclic
{

/// Timing relationship between the ros2_control write() loop and the WMX
/// communication cycle.
///
/// Both periods default to 0 (unset) so that a caller who forgets to read the
/// real cycle time gets a safe no-op interval rather than a plausible-looking
/// wrong one. The engine cycle time is not configured anywhere in this
/// workspace: read it from CoreMotionStatus::cycleTimeMilliseconds and never
/// hardcode it.
struct StreamTiming
{
  /// WMX communication cycle [ms], from CoreMotionStatus::cycleTimeMilliseconds.
  double cycle_ms = 0.0;

  /// Interval between push() calls [ms], i.e. the ros2_control write() period.
  double push_period_ms = 0.0;

  /// Proportional gain, in fractions of the nominal interval per command of
  /// depth error. The closed loop on queued cycles C is
  ///     C[k+1] = (1 - gain) * C[k] + gain * C_setpoint
  /// so it is stable for 0 < gain < 2 and deadbeat at 1.0. Kept below 1 to stay
  /// away from ringing when the loop period itself is jittering.
  ///
  /// Note that with a command-granular depth the `tolerance` clamp saturates
  /// before this gain has much say (see nextInterval), so `tolerance` is usually
  /// the more effective knob of the two.
  double depth_gain = 0.5;

  /// Largest fractional stretch/compress applied to one interval. A command
  /// stretched by this fraction is commanded that much slower over its segment,
  /// so this bounds the velocity error the regulator can introduce.
  double tolerance = 0.25;

  /// True when both periods are finite and positive and the gains are sane.
  bool valid() const
  {
    return std::isfinite(cycle_ms) && cycle_ms > 0.0 &&
           std::isfinite(push_period_ms) && push_period_ms > 0.0 &&
           std::isfinite(depth_gain) && depth_gain > 0.0 && depth_gain < 2.0 &&
           std::isfinite(tolerance) && tolerance >= 0.0 && tolerance <= 1.0;
  }

  /// Cycles one command must span for the buffer to drain at the rate it is fed.
  /// Returns 1 (a single-cycle command, no interpolation) when timing is unset,
  /// which holds position rather than moving unpredictably.
  unsigned int nominalInterval() const
  {
    if (!valid()) {return 1u;}
    const std::int64_t n = std::lround(push_period_ms / cycle_ms);
    return static_cast<unsigned int>(std::max<std::int64_t>(1, n));
  }

  /// Depth setpoint in *commands*: how many push-periods of runway to hold.
  /// Each command of runway buys tolerance to one late write() at the cost of
  /// one push-period of added latency. 2 is a reasonable starting point.
  unsigned int targetDepth(unsigned int periods = 2u) const
  {
    return std::max(1u, periods);
  }

  /// Buffer size to request from OpenCyclicBuffer, in *commands*.
  ///
  /// Note this is independent of cycle_ms: because a multi-cycle command still
  /// occupies one slot, the buffer is sized by how many commands are in flight,
  /// not by how much motion they represent. That is the whole payoff of
  /// intervalCycles, and it is easy to over-allocate by an order of magnitude if
  /// you reason in cycles instead.
  unsigned int bufferCommands(double horizon_s) const
  {
    if (!valid() || !std::isfinite(horizon_s) || horizon_s <= 0.0) {return 1u;}
    const std::int64_t n = std::lround(horizon_s * 1000.0 / push_period_ms);
    return static_cast<unsigned int>(std::max<std::int64_t>(1, n));
  }

  /// Approximate runway currently queued [ms], for logging. Each queued command
  /// carries about one push-period of motion.
  double runwayMs(unsigned int observed_depth) const
  {
    return static_cast<double>(observed_depth) * push_period_ms;
  }
};

/// intervalCycles for the next command, given the observed buffer depth.
///
/// Both depths are in *commands* (see the units note at the top of this file).
///   depth below setpoint -> stretch, so the queue drains slower and refills
///   depth above setpoint -> compress, so the queue drains faster and sheds lag
///
/// Because depth is a small integer and the correction is clamped to
/// +/- tolerance, this saturates for any error of one command or more: in
/// practice it behaves as "stretch by the maximum, hold, or compress by the
/// maximum", with the setpoint as a one-command deadband. That is deliberate --
/// finer resolution is not available, as the remaining cycles of the command
/// currently executing are not exposed by the API.
///
/// The result is always >= 1 and within +/- tolerance of the nominal interval,
/// for any input including an unset StreamTiming or a nonsense depth.
inline unsigned int nextInterval(
  const StreamTiming & timing, unsigned int target_depth, unsigned int observed_depth)
{
  const std::int64_t nominal = static_cast<std::int64_t>(timing.nominalInterval());
  const std::int64_t span = std::llround(static_cast<double>(nominal) * timing.tolerance);
  const std::int64_t lo = std::max<std::int64_t>(1, nominal - span);
  const std::int64_t hi = std::max<std::int64_t>(lo, nominal + span);

  // Widen to signed before subtracting: depths are unsigned and either may be
  // the larger, and observed_depth can legitimately exceed the setpoint.
  const std::int64_t error =
    static_cast<std::int64_t>(target_depth) - static_cast<std::int64_t>(observed_depth);

  // Scale the command-error into cycles via the nominal interval, so depth_gain
  // stays dimensionless and the loop analysis in its doc comment holds.
  const std::int64_t correction =
    std::llround(timing.depth_gain * static_cast<double>(nominal * error));

  return static_cast<unsigned int>(std::clamp(nominal + correction, lo, hi));
}

/// Tracks the real interval between push() calls, as handed to ros2_control's
/// write(time, period).
///
/// Why not just trust the configured update_rate: nominalInterval() is a ratio
/// of the loop period, so a 10% error in it is a permanent 10% bias on the
/// commanded velocity of every segment. The measured period is the truth.
///
/// Averages an EMA over the first `warmup_samples` observations, then freezes.
/// Freezing is the point: once running, one long write() (a preemption, a
/// planning-scene update) must not be read as "the loop got slower", or the
/// stream would stretch every command from then on and quietly accumulate lag.
class PeriodTracker
{
public:
  /// `seed_ms` is the expected period (1000 / update_rate), used until the first
  /// sample lands and as the reference for outlier rejection.
  explicit PeriodTracker(
    double seed_ms, unsigned int warmup_samples = 100u, double alpha = 0.1)
  : period_ms_(seed_ms), seed_ms_(seed_ms),
    warmup_samples_(warmup_samples), alpha_(alpha) {}

  /// Feed one measured period [ms]. Ignored once settled, and ignored for
  /// samples that are not finite and positive, or that are far enough from the
  /// seed to be a stall or clock glitch rather than the loop rate.
  void observe(double measured_ms)
  {
    if (settled() || !std::isfinite(measured_ms) || measured_ms <= 0.0) {return;}
    if (seed_ms_ > 0.0 && (measured_ms > 4.0 * seed_ms_ || measured_ms < 0.25 * seed_ms_)) {
      ++rejected_;
      return;
    }
    period_ms_ = (1.0 - alpha_) * period_ms_ + alpha_ * measured_ms;
    ++samples_;
  }

  double periodMs() const {return period_ms_;}
  bool settled() const {return samples_ >= warmup_samples_;}
  unsigned int samples() const {return samples_;}
  unsigned int rejected() const {return rejected_;}

private:
  double period_ms_;
  double seed_ms_;
  unsigned int warmup_samples_;
  double alpha_;
  unsigned int samples_ = 0u;
  unsigned int rejected_ = 0u;
};

}  // namespace cyclic
}  // namespace wmx_r2_control

#endif  // WMX_R2_CONTROL__CYCLIC_STREAM_TIMING_HPP_
