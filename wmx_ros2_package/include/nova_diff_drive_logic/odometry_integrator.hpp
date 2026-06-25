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

  /// Advance the pose by an exact arc over a single step given body displacement
  /// (ds [m], dtheta [rad]). dt-free — for encoder-position-delta dead reckoning,
  /// which is more precise than velocity*dt (no constant-velocity-over-dt
  /// assumption, no dt-jitter sensitivity). Branchless midpoint exact-arc using
  /// sinc, numerically stable and continuous through dtheta = 0:
  ///   x += ds*cos(theta + dtheta/2)*sinc(dtheta/2)
  ///   y += ds*sin(theta + dtheta/2)*sinc(dtheta/2)
  ///   theta += dtheta
  void integrateDelta(double ds, double dtheta)
  {
    if (!std::isfinite(ds) || !std::isfinite(dtheta)) {return;}  // ignore invalid steps
    const double half = 0.5 * dtheta;
    const double mid = pose_.theta + half;
    const double k = ds * sinc(half);
    pose_.x += k * std::cos(mid);
    pose_.y += k * std::sin(mid);
    pose_.theta += dtheta;
  }

  const Pose2D & pose() const {return pose_;}
  void reset(const Pose2D & p = {}) {pose_ = p;}

private:
  /// sin(a)/a with the removable singularity guarded (-> 1 as a -> 0), so the
  /// midpoint exact-arc in integrateDelta() is branchless and well-defined at
  /// dtheta = 0. Never evaluates a literal sin(0)/0.
  static double sinc(double a)
  {
    // 2-term Taylor for small |a| (error ~ a^4/120, far below double eps here);
    // direct sin(a)/a is accurate for all larger a.
    if (std::abs(a) < 1e-8) {return 1.0 - a * a / 6.0;}
    return std::sin(a) / a;
  }

  Pose2D pose_;
  double straight_eps_;
};

}  // namespace nova_diff_drive_logic

#endif  // NOVA_DIFF_DRIVE_LOGIC__ODOMETRY_INTEGRATOR_HPP_
