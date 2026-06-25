// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
#include <gtest/gtest.h>

#include <limits>

#include "differential_drive_controller.hpp"

using diff_drive::OdomDeltaAccumulator;

namespace
{
constexpr double kEps = 1e-9;
}

TEST(OdomDeltaAccumulator, AccumulatesAbsoluteTravel)
{
  OdomDeltaAccumulator acc;
  acc.accumulate({1.0, 0.5}, 1.0);   // +1.0 linear, +0.5 angular
  acc.accumulate({2.0, 0.5}, 0.5);   // +1.0 linear, +0.25 angular
  const auto d = acc.take();
  EXPECT_NEAR(d.linear, 2.0, kEps);
  EXPECT_NEAR(d.angular, 0.75, kEps);
}

TEST(OdomDeltaAccumulator, UsesAbsoluteValues)
{
  OdomDeltaAccumulator acc;
  acc.accumulate({-1.0, -2.0}, 1.0);  // magnitudes accumulate, not signed
  const auto d = acc.take();
  EXPECT_NEAR(d.linear, 1.0, kEps);
  EXPECT_NEAR(d.angular, 2.0, kEps);
}

TEST(OdomDeltaAccumulator, TakeResetsToZero)
{
  OdomDeltaAccumulator acc;
  acc.accumulate({1.0, 1.0}, 1.0);
  acc.take();
  EXPECT_NEAR(acc.peek().linear, 0.0, kEps);
  EXPECT_NEAR(acc.peek().angular, 0.0, kEps);
}

TEST(OdomDeltaAccumulator, IgnoresInvalidDt)
{
  OdomDeltaAccumulator acc;
  acc.accumulate({5.0, 5.0}, -1.0);                                    // negative dt
  acc.accumulate({5.0, 5.0}, std::numeric_limits<double>::quiet_NaN());  // NaN dt
  EXPECT_NEAR(acc.peek().linear, 0.0, kEps);
  EXPECT_NEAR(acc.peek().angular, 0.0, kEps);
}

// ---- accumulateDelta: direct body-displacement (dt-free) accumulation ----

TEST(OdomDeltaAccumulator, AccumulateDeltaAbsolute)
{
  OdomDeltaAccumulator acc;
  acc.accumulateDelta(1.0, 0.5);
  acc.accumulateDelta(-2.0, -0.5);  // magnitudes accumulate, not signed
  const auto d = acc.take();
  EXPECT_NEAR(d.linear, 3.0, kEps);
  EXPECT_NEAR(d.angular, 1.0, kEps);
}

TEST(OdomDeltaAccumulator, AccumulateDeltaIgnoresNonFinite)
{
  OdomDeltaAccumulator acc;
  acc.accumulateDelta(std::numeric_limits<double>::quiet_NaN(), 0.1);
  acc.accumulateDelta(1.0, std::numeric_limits<double>::infinity());
  EXPECT_NEAR(acc.peek().linear, 0.0, kEps);
  EXPECT_NEAR(acc.peek().angular, 0.0, kEps);
}
