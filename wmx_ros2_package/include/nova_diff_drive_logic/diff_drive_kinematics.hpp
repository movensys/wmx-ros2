// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
//
// WMX-free differential-drive kinematics, extracted from
// wmx_ros2_package/src/differential_drive_controller.cpp so it can be unit-tested
// without ROS, WMX, or hardware. The chosen controller (node or ros2_control)
// can call into this instead of duplicating the math inline.
#ifndef NOVA_DIFF_DRIVE_LOGIC__DIFF_DRIVE_KINEMATICS_HPP_
#define NOVA_DIFF_DRIVE_LOGIC__DIFF_DRIVE_KINEMATICS_HPP_

#include <cassert>

namespace nova_diff_drive_logic
{

/// Body (chassis) velocity: linear x [m/s], angular z [rad/s].
struct BodyVel
{
  double linear = 0.0;
  double angular = 0.0;
};

/// Per-wheel angular velocity [rad/s].
struct WheelOmega
{
  double left = 0.0;
  double right = 0.0;
};

/// Differential-drive geometry.
///   wheel_radius      R [m]
///   wheel_separation  L [m]  (a.k.a. "wheel_to_wheel" in the upstream node)
struct DiffDriveModel
{
  double wheel_radius = 0.0;
  double wheel_separation = 0.0;

  /// Inverse kinematics: body velocity -> wheel angular velocities [rad/s].
  ///   wl = (2v - wL) / (2R),   wr = (2v + wL) / (2R)
  WheelOmega inverse(const BodyVel & cmd) const
  {
    // Precondition: wheel_radius > 0. Config should be validated at the
    // param-load boundary (the controller); this assert catches misuse in dev.
    assert(wheel_radius > 0.0);
    return {
      (2.0 * cmd.linear - cmd.angular * wheel_separation) / (2.0 * wheel_radius),
      (2.0 * cmd.linear + cmd.angular * wheel_separation) / (2.0 * wheel_radius)};
  }

  /// Forward kinematics: wheel angular velocities -> body velocity.
  ///   v = R(wr + wl) / 2,   w = R(wr - wl) / L
  BodyVel forward(const WheelOmega & omega) const
  {
    assert(wheel_radius > 0.0 && wheel_separation > 0.0);  // see inverse()
    return {
      (omega.right * wheel_radius + omega.left * wheel_radius) / 2.0,
      (omega.right * wheel_radius - omega.left * wheel_radius) / wheel_separation};
  }

  /// Forward kinematics over a single time *step* rather than instantaneously.
  /// Because forward() is a linear map with no dt term, feeding per-wheel angle
  /// deltas [rad] yields body deltas directly: the returned BodyVel carries
  ///   .linear = Δs [m]   (chassis displacement over the step)
  ///   .angular = Δθ [rad] (heading change over the step)
  /// Same math as forward(); separate name so call sites read in the right units.
  BodyVel forwardDelta(double d_phi_left, double d_phi_right) const
  {
    // DiffDriveModel::forward, not std::forward (cpplint IWYU false positive):
    return forward({d_phi_left, d_phi_right});  // NOLINT(build/include_what_you_use)
  }
};

}  // namespace nova_diff_drive_logic

#endif  // NOVA_DIFF_DRIVE_LOGIC__DIFF_DRIVE_KINEMATICS_HPP_
