// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "differential_drive_controller.hpp"

using diff_drive::BodyVel;
using diff_drive::OdometryIntegrator;

namespace
{
constexpr double kEps = 1e-9;
}

TEST(OdometryIntegrator, StartsAtOrigin)
{
  OdometryIntegrator odom;
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}

TEST(OdometryIntegrator, StraightLineAlongX)
{
  OdometryIntegrator odom;
  odom.odometryPoseCalculation({1.0, 0.0}, 2.0);  // 1 m/s for 2 s
  EXPECT_NEAR(odom.pose().x, 2.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}

TEST(OdometryIntegrator, RotateInPlace)
{
  OdometryIntegrator odom;
  odom.odometryPoseCalculation({0.0, 1.0}, 1.5);  // no translation
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 1.5, kEps);
}

TEST(OdometryIntegrator, ExactQuarterArc)
{
  // Constant v=1, w=pi/2 for dt=1 traces an exact quarter circle of radius 2/pi.
  OdometryIntegrator odom;
  const double w = M_PI / 2.0;
  odom.odometryPoseCalculation({1.0, w}, 1.0);
  const double r = 1.0 / w;  // = 2/pi
  EXPECT_NEAR(odom.pose().x, r, 1e-9);          // r*(sin(pi/2)-sin(0))
  EXPECT_NEAR(odom.pose().y, r, 1e-9);          // -r*(cos(pi/2)-cos(0))
  EXPECT_NEAR(odom.pose().theta, M_PI / 2.0, 1e-9);
}

TEST(OdometryIntegrator, ResetClearsPose)
{
  OdometryIntegrator odom;
  odom.odometryPoseCalculation({1.0, 1.0}, 1.0);
  odom.reset();
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}

TEST(OdometryIntegrator, IgnoresInvalidDt)
{
  OdometryIntegrator odom;
  odom.odometryPoseCalculation({1.0, 1.0}, 0.0);
  odom.odometryPoseCalculation({1.0, 1.0}, -0.5);
  odom.odometryPoseCalculation({1.0, 1.0}, std::numeric_limits<double>::quiet_NaN());
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}

// ---- odometryPoseCalculation(ds, dtheta): position-delta (dt-free) dead reckoning ----

TEST(OdometryIntegrator, IntegrateDeltaStraight)
{
  OdometryIntegrator odom;
  odom.odometryPoseCalculation(2.0, 0.0);  // 2 m forward, no turn
  EXPECT_NEAR(odom.pose().x, 2.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}

TEST(OdometryIntegrator, IntegrateDeltaRotateInPlace)
{
  OdometryIntegrator odom;
  odom.odometryPoseCalculation(0.0, 1.5);  // turn only
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 1.5, kEps);
}

TEST(OdometryIntegrator, ExactQuarterArcDelta)
{
  // Same quarter circle as ExactQuarterArc, expressed as one body-displacement
  // step: ds = v*dt = 1, dtheta = w*dt = pi/2. The dt-free sinc form must land on
  // the same pose (radius r = 2/pi) as the velocity-based odometryPoseCalculation().
  OdometryIntegrator odom;
  const double w = M_PI / 2.0;
  odom.odometryPoseCalculation(1.0, w);
  const double r = 1.0 / w;  // 2/pi
  EXPECT_NEAR(odom.pose().x, r, kEps);
  EXPECT_NEAR(odom.pose().y, r, kEps);
  EXPECT_NEAR(odom.pose().theta, M_PI / 2.0, kEps);

  // Cross-check: identical pose to the velocity path over the same motion.
  OdometryIntegrator vel;
  vel.odometryPoseCalculation({1.0, w}, 1.0);
  EXPECT_NEAR(odom.pose().x, vel.pose().x, kEps);
  EXPECT_NEAR(odom.pose().y, vel.pose().y, kEps);
  EXPECT_NEAR(odom.pose().theta, vel.pose().theta, kEps);
}

TEST(OdometryIntegrator, IntegrateDeltaContinuousNearZeroTheta)
{
  // sinc must stay finite/accurate as dtheta -> 0 (never sin(0)/0). A tiny turn
  // with forward motion is ~ a straight line of length ds.
  OdometryIntegrator odom;
  odom.odometryPoseCalculation(1.0, 1e-12);
  EXPECT_NEAR(odom.pose().x, 1.0, 1e-9);
  EXPECT_NEAR(odom.pose().y, 0.0, 1e-9);
  EXPECT_NEAR(odom.pose().theta, 1e-12, kEps);
}

TEST(OdometryIntegrator, IntegrateDeltaIgnoresNonFinite)
{
  OdometryIntegrator odom;
  odom.odometryPoseCalculation(std::numeric_limits<double>::quiet_NaN(), 0.1);
  odom.odometryPoseCalculation(1.0, std::numeric_limits<double>::infinity());
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}
