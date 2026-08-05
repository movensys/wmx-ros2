// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// Unit tests for the WMX-free cyclic-buffer stream timing logic. No WMX3 SDK
// required, so these run on any machine and in CI.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>

#include "wmx_r2_control/cyclic_stream_timing.hpp"

using wmx_r2_control::cyclic::StreamTiming;
using wmx_r2_control::cyclic::nextInterval;

namespace
{

/// Default rig: 0.5 ms engine cycle, 100 Hz ros2_control loop.
/// nominal = 10 / 0.5 = 20 cycles, span = 25% = 5, so the band is [15, 25].
constexpr double kCycleMs = 0.5;
constexpr double kPushMs = 10.0;
constexpr unsigned int kNominal = 20u;
constexpr unsigned int kLo = 15u;
constexpr unsigned int kHi = 25u;

StreamTiming makeTiming(double cycle_ms = kCycleMs, double push_period_ms = kPushMs)
{
  StreamTiming t;
  t.cycle_ms = cycle_ms;
  t.push_period_ms = push_period_ms;
  return t;
}

}  // namespace

// ===========================================================================
// nominalInterval
// ===========================================================================

TEST(StreamTiming, NominalIntervalDividesPushPeriodByCycle)
{
  EXPECT_EQ(makeTiming(0.5, 10.0).nominalInterval(), 20u);   // 100 Hz loop
  EXPECT_EQ(makeTiming(0.5, 25.0).nominalInterval(), 50u);   // Servo's 40 Hz
  EXPECT_EQ(makeTiming(1.0, 10.0).nominalInterval(), 10u);   // 1 ms engine cycle
  EXPECT_EQ(makeTiming(0.25, 10.0).nominalInterval(), 40u);  // 0.25 ms engine cycle
  EXPECT_EQ(makeTiming(0.5, 2.0).nominalInterval(), 4u);     // 500 Hz loop
}

TEST(StreamTiming, NominalIntervalNeverZero)
{
  // A push period shorter than the engine cycle cannot be honoured; the command
  // must still span at least one cycle.
  EXPECT_EQ(makeTiming(10.0, 2.0).nominalInterval(), 1u);
  EXPECT_EQ(makeTiming(0.5, 0.4).nominalInterval(), 1u);
  EXPECT_EQ(makeTiming(0.5, 0.5).nominalInterval(), 1u);
}

// ===========================================================================
// Validation: an unset or nonsense StreamTiming must degrade to a hold, never
// to a plausible-looking wrong interval.
// ===========================================================================

TEST(StreamTiming, DefaultConstructedIsInvalid)
{
  const StreamTiming t;  // periods unset
  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.nominalInterval(), 1u);
  EXPECT_EQ(t.bufferCommands(5.0), 1u);
}

TEST(StreamTiming, RejectsNonFiniteAndNonPositivePeriods)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(makeTiming(nan, 10.0).valid());
  EXPECT_FALSE(makeTiming(0.5, nan).valid());
  EXPECT_FALSE(makeTiming(inf, 10.0).valid());
  EXPECT_FALSE(makeTiming(0.0, 10.0).valid());   // engine not communicating yet
  EXPECT_FALSE(makeTiming(-0.5, 10.0).valid());
  EXPECT_FALSE(makeTiming(0.5, 0.0).valid());
  EXPECT_TRUE(makeTiming().valid());
}

TEST(StreamTiming, RejectsGainsOutsideTheStableRange)
{
  StreamTiming t = makeTiming();
  t.depth_gain = 0.0;
  EXPECT_FALSE(t.valid());
  t.depth_gain = 2.0;  // loop is marginally stable at 2, unstable above
  EXPECT_FALSE(t.valid());
  t.depth_gain = 1.0;  // deadbeat, still valid
  EXPECT_TRUE(t.valid());

  t = makeTiming();
  t.tolerance = -0.1;
  EXPECT_FALSE(t.valid());
  t.tolerance = 1.5;
  EXPECT_FALSE(t.valid());
  t.tolerance = 0.0;  // no adjustment allowed, but well-defined
  EXPECT_TRUE(t.valid());
}

TEST(StreamTiming, InvalidTimingYieldsSingleCycleCommands)
{
  const StreamTiming t;
  for (unsigned int depth = 0; depth < 10u; ++depth) {
    EXPECT_EQ(nextInterval(t, 2u, depth), 1u) << "depth=" << depth;
  }
}

// ===========================================================================
// targetDepth / bufferCommands
// ===========================================================================

TEST(StreamTiming, TargetDepthIsCommandsOfRunway)
{
  const StreamTiming t = makeTiming();
  EXPECT_EQ(t.targetDepth(), 2u);    // default
  EXPECT_EQ(t.targetDepth(4u), 4u);
  EXPECT_EQ(t.targetDepth(0u), 1u);  // zero runway would underrun by design
}

TEST(StreamTiming, BufferCommandsCountsCommandsNotCycles)
{
  // 5 s at 10 ms per command = 500 commands, whatever the engine cycle is:
  // intervalCycles does not change a command's one-slot footprint.
  EXPECT_EQ(makeTiming(0.5, 10.0).bufferCommands(5.0), 500u);
  EXPECT_EQ(makeTiming(0.25, 10.0).bufferCommands(5.0), 500u);
  EXPECT_EQ(makeTiming(1.0, 10.0).bufferCommands(5.0), 500u);

  // Reasoning in cycles instead would ask the engine for 10x this at 0.25 ms.
  EXPECT_EQ(
    makeTiming(0.25, 10.0).bufferCommands(5.0),
    makeTiming(1.0, 10.0).bufferCommands(5.0));
}

TEST(StreamTiming, BufferCommandsRejectsNonsenseHorizons)
{
  const StreamTiming t = makeTiming();
  EXPECT_EQ(t.bufferCommands(0.0), 1u);
  EXPECT_EQ(t.bufferCommands(-5.0), 1u);
  EXPECT_EQ(t.bufferCommands(std::numeric_limits<double>::quiet_NaN()), 1u);
}

TEST(StreamTiming, RunwayMsScalesWithDepth)
{
  const StreamTiming t = makeTiming();
  EXPECT_DOUBLE_EQ(t.runwayMs(0u), 0.0);
  EXPECT_DOUBLE_EQ(t.runwayMs(2u), 20.0);
  EXPECT_DOUBLE_EQ(t.runwayMs(5u), 50.0);
}

// ===========================================================================
// nextInterval: direction, clamping, monotonicity
// ===========================================================================

TEST(NextInterval, AtSetpointReturnsNominal)
{
  EXPECT_EQ(nextInterval(makeTiming(), 2u, 2u), kNominal);
  EXPECT_EQ(nextInterval(makeTiming(), 5u, 5u), kNominal);
}

// The sign here is the bug that would produce a runaway: stretching when the
// buffer is already long would grow the queue without bound.
TEST(NextInterval, BelowSetpointStretches)
{
  const StreamTiming t = makeTiming();
  EXPECT_GT(nextInterval(t, 2u, 1u), kNominal);
  EXPECT_GT(nextInterval(t, 2u, 0u), kNominal);
  // Saturates at the clamp for a one-command error: 0.5 * 20 * 1 = 10 -> +5.
  EXPECT_EQ(nextInterval(t, 2u, 1u), kHi);
  EXPECT_EQ(nextInterval(t, 2u, 0u), kHi);
}

TEST(NextInterval, AboveSetpointCompresses)
{
  const StreamTiming t = makeTiming();
  EXPECT_LT(nextInterval(t, 2u, 3u), kNominal);
  EXPECT_LT(nextInterval(t, 2u, 50u), kNominal);
  EXPECT_EQ(nextInterval(t, 2u, 3u), kLo);
  EXPECT_EQ(nextInterval(t, 2u, 50u), kLo);
}

TEST(NextInterval, AlwaysWithinToleranceBandAndNeverZero)
{
  const StreamTiming t = makeTiming();
  for (unsigned int depth = 0; depth <= 1000u; ++depth) {
    const unsigned int interval = nextInterval(t, 2u, depth);
    EXPECT_GE(interval, kLo) << "depth=" << depth;
    EXPECT_LE(interval, kHi) << "depth=" << depth;
    EXPECT_GT(interval, 0u) << "depth=" << depth;
  }
}

TEST(NextInterval, HandlesExtremeDepthsWithoutOverflow)
{
  const StreamTiming t = makeTiming();
  EXPECT_EQ(nextInterval(t, 2u, std::numeric_limits<unsigned int>::max()), kLo);
  EXPECT_EQ(nextInterval(t, std::numeric_limits<unsigned int>::max(), 0u), kHi);
}

TEST(NextInterval, IsMonotonicNonIncreasingInDepth)
{
  const StreamTiming t = makeTiming();
  unsigned int previous = nextInterval(t, 3u, 0u);
  for (unsigned int depth = 1u; depth <= 50u; ++depth) {
    const unsigned int interval = nextInterval(t, 3u, depth);
    EXPECT_LE(interval, previous) << "depth=" << depth;
    previous = interval;
  }
}

// When the loop period equals the engine cycle there is no room to interpolate,
// so the regulator has no authority at all. Documented as a test so nobody tunes
// against a knob that cannot move.
TEST(NextInterval, HasNoAuthorityWhenNominalIntervalIsOne)
{
  const StreamTiming t = makeTiming(0.5, 0.5);
  ASSERT_EQ(t.nominalInterval(), 1u);
  EXPECT_EQ(nextInterval(t, 2u, 0u), 1u);
  EXPECT_EQ(nextInterval(t, 2u, 9u), 1u);
}

// ===========================================================================
// Closed loop against a FIFO model of the buffer
// ===========================================================================

namespace
{

struct SimResult
{
  unsigned int min_depth = std::numeric_limits<unsigned int>::max();
  unsigned int max_depth = 0u;
  unsigned int final_depth = 0u;
  int underruns = 0;      ///< pushes after which the buffer ran dry (axis held)
  double mean_depth = 0.0;
};

/// Simulate the real buffer: a FIFO of commands, each with its own remaining
/// cycle count, drained by however many cycles elapse between pushes.
///
/// `initial_depth` is how many commands are queued before the first push, which
/// start() normally primes to `target` but which can be higher after a burst of
/// late writes. `regulate == false` pushes a fixed nominal interval, i.e. the
/// naive implementation with no depth control.
SimResult simulate(
  const StreamTiming & timing, unsigned int target, unsigned int initial_depth, int pushes,
  double jitter_frac, std::uint32_t seed, bool regulate)
{
  const unsigned int nominal = timing.nominalInterval();
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jitter(1.0 - jitter_frac, 1.0 + jitter_frac);

  std::deque<std::int64_t> queue(initial_depth, static_cast<std::int64_t>(nominal));

  SimResult result;
  double sum = 0.0;

  for (int k = 0; k < pushes; ++k) {
    // push() reads the depth, then appends -- same order as the real call.
    const unsigned int depth = static_cast<unsigned int>(queue.size());
    result.min_depth = std::min(result.min_depth, depth);
    result.max_depth = std::max(result.max_depth, depth);
    sum += depth;

    queue.push_back(
      regulate ? nextInterval(timing, target, depth) : nominal);

    // Consume the cycles that elapse before the next push lands.
    std::int64_t budget = std::llround(nominal * jitter(rng));
    while (budget > 0 && !queue.empty()) {
      const std::int64_t take = std::min(budget, queue.front());
      queue.front() -= take;
      budget -= take;
      if (queue.front() == 0) {queue.pop_front();}
    }
    if (queue.empty()) {++result.underruns;}
  }

  result.final_depth = static_cast<unsigned int>(queue.size());
  result.mean_depth = sum / pushes;
  return result;
}

}  // namespace

TEST(ClosedLoop, HoldsDepthWithoutUnderrunUnderModerateJitter)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth();  // 2 commands
  const SimResult r = simulate(t, target, target, 2000, 0.20, 1234u, true);

  EXPECT_EQ(r.underruns, 0);
  EXPECT_GE(r.min_depth, 1u);
  // Settles into a one-command band around the setpoint: the deadband means it
  // dithers between target-1 and target+1, averaging the setpoint.
  EXPECT_LE(r.max_depth, target + 1u);
  EXPECT_NEAR(r.mean_depth, static_cast<double>(target), 0.25);
}

TEST(ClosedLoop, SurvivesSevereJitterWithMoreRunway)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth(4u);
  const SimResult r = simulate(t, target, target, 2000, 0.50, 4321u, true);

  EXPECT_EQ(r.underruns, 0);
  EXPECT_GE(r.min_depth, 1u);
  EXPECT_LE(r.max_depth, target + 4u);
}

// The regulator has to earn its place. With a fixed nominal interval the queued
// cycle count is a zero-drift random walk, so the buffer wanders off in one of
// two directions and which one you get depends on the noise: down to empty (the
// axis stalls, then steps) or up without bound (commanded motion falls further
// and further behind the operator). Both are disqualifying, so assert across
// several seeds rather than pinning one seed to one failure mode.
TEST(ClosedLoop, FixedIntervalDriftsWhereRegulatedHolds)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth();
  const std::uint32_t seeds[] = {99u, 1234u, 7u, 555u};

  int fixed_underruns = 0;
  unsigned int fixed_worst_depth = 0;
  unsigned int regulated_worst_depth = 0;

  for (const std::uint32_t seed : seeds) {
    const SimResult fixed = simulate(t, target, target, 2000, 0.20, seed, false);
    const SimResult regulated = simulate(t, target, target, 2000, 0.20, seed, true);

    fixed_underruns += fixed.underruns;
    fixed_worst_depth = std::max(fixed_worst_depth, fixed.max_depth);
    regulated_worst_depth = std::max(regulated_worst_depth, regulated.max_depth);

    // The regulated loop must be well-behaved on *every* seed.
    EXPECT_EQ(regulated.underruns, 0) << "seed=" << seed;
    EXPECT_LE(regulated.max_depth, target + 1u) << "seed=" << seed;
  }

  // Underrun failure mode: some seeds drain the buffer dry.
  EXPECT_GT(fixed_underruns, 0);
  // Latency failure mode: others pile up many push-periods of stale commands.
  EXPECT_GE(fixed_worst_depth, 3u * regulated_worst_depth);
}

// A burst of late writes leaves the queue deep, which is pure added latency:
// every queued command must execute before the operator's newest target does.
// The regulator has to shed it, not settle for it.
TEST(ClosedLoop, DrainsAnOverfilledBufferBackToSetpoint)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth();
  const SimResult r = simulate(t, target, target + 20u, 2000, 0.0, 7u, true);

  EXPECT_EQ(r.underruns, 0);
  EXPECT_LE(r.final_depth, target + 1u) << "regulator failed to shed latency";
  EXPECT_EQ(r.max_depth, target + 20u) << "depth should only ever fall from the start";
}

// The mirror case: starting empty, the stretch must fill the buffer to the
// setpoint rather than limping along one command deep forever.
TEST(ClosedLoop, FillsAnEmptyBufferUpToSetpoint)
{
  const StreamTiming t = makeTiming();
  const unsigned int target = t.targetDepth(4u);
  const SimResult r = simulate(t, target, 0u, 2000, 0.0, 8u, true);

  EXPECT_GE(r.final_depth, target) << "regulator failed to build runway";
}
