// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
//
// Dead-reckoning pose integration from body velocity. Ported from
// wmx-ros2 differential_drive_controller (compute_odometry) — exact arc
// integration for non-zero yaw rate, straight-line otherwise. WMX/ROS-free.
#ifndef NOVA_DIFF_DRIVE_LOGIC__ODOMETRY_INTEGRATOR_HPP_
#define NOVA_DIFF_DRIVE_LOGIC__ODOMETRY_INTEGRATOR_HPP_

#include <cmath>

#include "nova_diff_drive_logic/diff_drive_kinematics.hpp"

namespace nova_diff_drive_logic
{

/// 2D pose [m, m, rad].
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// Integrates body velocity into a pose. Stateful (accumulates x, y, theta).
class OdometryIntegrator
{
public:
  /// Below this |angular| [rad/s] we treat motion as a straight line to avoid
  /// dividing by ~0 (radius = v / w). Matches the upstream 0.001 threshold.
  explicit OdometryIntegrator(double straight_eps = 1e-3)
  : straight_eps_(straight_eps) {}

  /// Advance the pose by integrating `vel` over `dt` seconds.
  void integrate(const BodyVel & vel, double dt)
  {
    if (!std::isfinite(dt) || dt <= 0.0) {return;}  // ignore invalid/zero timesteps
    const double next_theta = pose_.theta + vel.angular * dt;
    // Threshold on |angular| (NOT |angular*dt|): the arc update below is exact
    // for any non-zero yaw rate over the step; the straight-line branch only
    // exists to avoid the radius = v/w blow-up as w -> 0. (Matches upstream and
    // ros2 diff_drive_controller convention.)
    if (std::abs(vel.angular) < straight_eps_) {
      const double dist = vel.linear * dt;
      pose_.x += dist * std::cos(pose_.theta);
      pose_.y += dist * std::sin(pose_.theta);
    } else {
      const double radius = vel.linear / vel.angular;
      pose_.x += radius * (std::sin(next_theta) - std::sin(pose_.theta));
      pose_.y -= radius * (std::cos(next_theta) - std::cos(pose_.theta));
    }
    pose_.theta = next_theta;
  }

  const Pose2D & pose() const {return pose_;}
  void reset(const Pose2D & p = {}) {pose_ = p;}

private:
  Pose2D pose_;
  double straight_eps_;
};

}  // namespace nova_diff_drive_logic

#endif  // NOVA_DIFF_DRIVE_LOGIC__ODOMETRY_INTEGRATOR_HPP_
