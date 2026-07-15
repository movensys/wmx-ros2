// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
#include <gtest/gtest.h>

#include "differential_drive_controller.hpp"

using diff_drive::BodyVel;
using diff_drive::DiffDriveModel;
using diff_drive::WheelOmega;

namespace
{
constexpr double kEps = 1e-9;
// Defaults from differential_drive_controller.cpp (wheel_radius, wheel_to_wheel).
DiffDriveModel model() {return DiffDriveModel{0.095, 0.55};}
}  // namespace

TEST(DiffDriveKinematics, ForwardZeroIsZero)
{
  const auto b = model().forward({0.0, 0.0});
  EXPECT_NEAR(b.linear, 0.0, kEps);
  EXPECT_NEAR(b.angular, 0.0, kEps);
}

TEST(DiffDriveKinematics, ForwardStraight)
{
  // Equal wheel speeds -> pure linear motion, no rotation.
  const auto m = model();
  const auto b = m.forward({2.0, 2.0});
  EXPECT_NEAR(b.linear, 2.0 * m.wheel_radius, kEps);
  EXPECT_NEAR(b.angular, 0.0, kEps);
}

TEST(DiffDriveKinematics, ForwardPureRotation)
{
  // Opposite wheel speeds -> pure rotation, no translation.
  const auto m = model();
  const auto b = m.forward({-3.0, 3.0});
  EXPECT_NEAR(b.linear, 0.0, kEps);
  EXPECT_NEAR(b.angular, 6.0 * m.wheel_radius / m.wheel_separation, kEps);
}

TEST(DiffDriveKinematics, InverseStraight)
{
  const auto m = model();
  const auto w = m.inverse({0.5, 0.0});
  EXPECT_NEAR(w.left, 0.5 / m.wheel_radius, kEps);
  EXPECT_NEAR(w.right, 0.5 / m.wheel_radius, kEps);
}

TEST(DiffDriveKinematics, InversePureRotation)
{
  const auto m = model();
  const auto w = m.inverse({0.0, 1.0});
  EXPECT_NEAR(w.left, -1.0 * m.wheel_separation / (2.0 * m.wheel_radius), kEps);
  EXPECT_NEAR(w.right, 1.0 * m.wheel_separation / (2.0 * m.wheel_radius), kEps);
}

TEST(DiffDriveKinematics, RoundTripBodyToWheelToBody)
{
  const auto m = model();
  const BodyVel cases[] = {
    {0.0, 0.0}, {1.0, 0.0}, {0.0, 0.8}, {0.6, -0.4}, {-0.3, 0.5}};
  for (const auto & in : cases) {
    const auto out = m.forward(m.inverse(in));
    EXPECT_NEAR(out.linear, in.linear, kEps);
    EXPECT_NEAR(out.angular, in.angular, kEps);
  }
}

// ---- forwardDelta: per-step wheel angle deltas [rad] -> body deltas {ds, dtheta} ----

TEST(DiffDriveKinematics, ForwardDeltaStraight)
{
  // Equal wheel angle deltas -> pure forward displacement, no heading change.
  const auto m = model();
  const auto d = m.forwardDelta(2.0, 2.0);
  EXPECT_NEAR(d.linear, 2.0 * m.wheel_radius, kEps);
  EXPECT_NEAR(d.angular, 0.0, kEps);
}

TEST(DiffDriveKinematics, ForwardDeltaPureRotation)
{
  const auto m = model();
  const auto d = m.forwardDelta(-3.0, 3.0);
  EXPECT_NEAR(d.linear, 0.0, kEps);
  EXPECT_NEAR(d.angular, 6.0 * m.wheel_radius / m.wheel_separation, kEps);
}

TEST(DiffDriveKinematics, ForwardDeltaMatchesForward)
{
  // forwardDelta is forward() reinterpreted over a step; they must agree numerically.
  const auto m = model();
  const double dl[] = {0.0, 2.0, -3.0, 1.5, 0.2};
  const double dr[] = {0.0, 2.0, 3.0, -0.7, 0.9};
  for (int i = 0; i < 5; ++i) {
    const auto d = m.forwardDelta(dl[i], dr[i]);
    const auto f = m.forward({dl[i], dr[i]});
    EXPECT_NEAR(d.linear, f.linear, kEps);
    EXPECT_NEAR(d.angular, f.angular, kEps);
  }
}
