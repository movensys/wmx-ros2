// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
#include <gtest/gtest.h>

#include "nova_diff_drive_logic/diff_drive_kinematics.hpp"

using nova_diff_drive_logic::BodyVel;
using nova_diff_drive_logic::DiffDriveModel;
using nova_diff_drive_logic::WheelOmega;

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
