// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
#include <gtest/gtest.h>

#include <limits>

#include "differential_drive_controller.hpp"

using diff_drive::AccelEstimator;

namespace
{
constexpr double kEps = 1e-9;
constexpr double kAlpha = 0.3;
}

TEST(AccelEstimator, FirstSampleIsZero)
{
  AccelEstimator est(kAlpha);
  const auto a = est.update({1.0, 1.0}, 0.1);  // first sample only primes baseline
  EXPECT_NEAR(a.linear, 0.0, kEps);
  EXPECT_NEAR(a.angular, 0.0, kEps);
}

TEST(AccelEstimator, LinearAccelEmaFirstStep)
{
  AccelEstimator est(kAlpha);
  est.update({0.0, 0.0}, 0.1);                 // prime
  const auto a = est.update({1.0, 0.0}, 0.1);  // raw = 10, EMA = 0.3*10
  EXPECT_NEAR(a.linear, kAlpha * 10.0, 1e-9);
  EXPECT_NEAR(a.angular, 0.0, kEps);           // angular stopped -> clamped 0
}

TEST(AccelEstimator, AngularAccelEmaFirstStep)
{
  AccelEstimator est(kAlpha);
  est.update({0.0, 0.0}, 0.1);                 // prime
  const auto a = est.update({0.0, 2.0}, 0.1);  // raw = 20, EMA = 0.3*20
  EXPECT_NEAR(a.linear, 0.0, kEps);
  EXPECT_NEAR(a.angular, kAlpha * 20.0, 1e-9);
}

TEST(AccelEstimator, StoppedSnapsToZero)
{
  AccelEstimator est(kAlpha);
  est.update({0.0, 0.0}, 0.1);                 // prime
  const auto a = est.update({0.0, 0.0}, 0.1);  // both samples ~0 -> snapped to 0
  EXPECT_NEAR(a.linear, 0.0, kEps);
  EXPECT_NEAR(a.angular, 0.0, kEps);
}

TEST(AccelEstimator, NonPositiveDtRebaselines)
{
  AccelEstimator est(kAlpha);
  est.update({0.0, 0.0}, 0.1);                 // prime
  const auto a = est.update({5.0, 0.0}, 0.0);  // dt<=0 -> no derivative, zero
  EXPECT_NEAR(a.linear, 0.0, kEps);
  EXPECT_NEAR(a.angular, 0.0, kEps);
}

TEST(AccelEstimator, NanDtIsZero)
{
  AccelEstimator est(kAlpha);
  est.update({0.0, 0.0}, 0.1);  // prime
  const auto a = est.update({5.0, 0.0}, std::numeric_limits<double>::quiet_NaN());
  EXPECT_NEAR(a.linear, 0.0, kEps);  // NaN dt must not poison the estimate
  EXPECT_NEAR(a.angular, 0.0, kEps);
}
