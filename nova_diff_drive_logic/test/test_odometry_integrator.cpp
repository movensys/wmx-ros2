// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "nova_diff_drive_logic/odometry_integrator.hpp"

using nova_diff_drive_logic::BodyVel;
using nova_diff_drive_logic::OdometryIntegrator;

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
  odom.integrate({1.0, 0.0}, 2.0);  // 1 m/s for 2 s
  EXPECT_NEAR(odom.pose().x, 2.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}

TEST(OdometryIntegrator, RotateInPlace)
{
  OdometryIntegrator odom;
  odom.integrate({0.0, 1.0}, 1.5);  // no translation
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 1.5, kEps);
}

TEST(OdometryIntegrator, ExactQuarterArc)
{
  // Constant v=1, w=pi/2 for dt=1 traces an exact quarter circle of radius 2/pi.
  OdometryIntegrator odom;
  const double w = M_PI / 2.0;
  odom.integrate({1.0, w}, 1.0);
  const double r = 1.0 / w;  // = 2/pi
  EXPECT_NEAR(odom.pose().x, r, 1e-9);          // r*(sin(pi/2)-sin(0))
  EXPECT_NEAR(odom.pose().y, r, 1e-9);          // -r*(cos(pi/2)-cos(0))
  EXPECT_NEAR(odom.pose().theta, M_PI / 2.0, 1e-9);
}

TEST(OdometryIntegrator, ResetClearsPose)
{
  OdometryIntegrator odom;
  odom.integrate({1.0, 1.0}, 1.0);
  odom.reset();
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}

TEST(OdometryIntegrator, IgnoresInvalidDt)
{
  OdometryIntegrator odom;
  odom.integrate({1.0, 1.0}, 0.0);
  odom.integrate({1.0, 1.0}, -0.5);
  odom.integrate({1.0, 1.0}, std::numeric_limits<double>::quiet_NaN());
  EXPECT_NEAR(odom.pose().x, 0.0, kEps);
  EXPECT_NEAR(odom.pose().y, 0.0, kEps);
  EXPECT_NEAR(odom.pose().theta, 0.0, kEps);
}
