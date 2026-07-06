// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
//
// WMX/ROS-free differential-drive logic for the standalone
// differential_drive_controller node: kinematics, dead-reckoning odometry,
// odometry-delta accumulation, and body-acceleration estimation. Kept free of
// ROS, WMX, and hardware dependencies so it can be unit-tested in isolation; the
// controller node is the thin ROS/WMX wiring around these pure components.
#ifndef DIFFERENTIAL_DRIVE_CONTROLLER_HPP_
#define DIFFERENTIAL_DRIVE_CONTROLLER_HPP_

#include <cassert>
#include <cmath>

namespace diff_drive
{

// ===========================================================================
// Kinematics
// ===========================================================================

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

// ===========================================================================
// Dead-reckoning odometry
// ===========================================================================

/// 2D pose [m, m, rad].
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// Integrates body velocity (or a per-step body displacement) into a pose.
/// Stateful (accumulates x, y, theta).
class OdometryIntegrator
{
public:
  /// Below this |angular| [rad/s] we treat motion as a straight line to avoid
  /// dividing by ~0 (radius = v / w). Matches the upstream 0.001 threshold.
  explicit OdometryIntegrator(double straight_eps = 1e-3)
  : straight_eps_(straight_eps) {}

  /// Advance the pose by integrating `vel` over `dt` seconds.
  void odometryPoseCalculation(const BodyVel & vel, double dt)
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
  void odometryPoseCalculation(double ds, double dtheta)
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
  /// midpoint exact-arc in odometryPoseCalculation() is branchless and well-defined at
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

// ===========================================================================
// Odometry-delta accumulation (for the /odom_deltas topic)
// ===========================================================================

/// Accumulated absolute travel since the last take(): linear [m], angular [rad].
struct OdomDelta
{
  double linear = 0.0;
  double angular = 0.0;
};

/// Accumulates absolute travel between reads, reset on publish via take(). Two
/// overloads: odometryDeltaAccumulation(vel, dt) [velocity path, |v|*dt / |w|*dt]
/// and odometryDeltaAccumulation(ds, dtheta) [position-delta path, |ds| / |dtheta|].
/// The controller drives the position-delta path; both share the same semantics.
class OdomDeltaAccumulator
{
public:
  void odometryDeltaAccumulation(const BodyVel & vel, double dt)
  {
    if (!std::isfinite(dt) || dt <= 0.0) {return;}  // ignore invalid/negative dt
    delta_.linear += std::abs(vel.linear * dt);
    delta_.angular += std::abs(vel.angular * dt);
  }

  /// Accumulate absolute body displacement (ds [m], dtheta [rad]) directly — for
  /// encoder-position-delta dead reckoning (dt-free; more exact than |v|*dt).
  void odometryDeltaAccumulation(double ds, double dtheta)
  {
    if (!std::isfinite(ds) || !std::isfinite(dtheta)) {return;}  // ignore invalid steps
    delta_.linear += std::abs(ds);
    delta_.angular += std::abs(dtheta);
  }

  /// Return accumulated deltas and reset the accumulator to zero.
  OdomDelta take()
  {
    const OdomDelta out = delta_;
    delta_ = {};
    return out;
  }

  const OdomDelta & peek() const {return delta_;}

private:
  OdomDelta delta_;
};

// ===========================================================================
// Body-acceleration estimation (for the /odom_accel topic)
// ===========================================================================
//
// EMA-filtered acceleration estimator: raw derivative of body velocity,
// EMA-smoothed, snapped to zero when both current and previous velocity are ~0.
// The node layer owns publish rate-limiting (1/accel_publish_rate); this class
// is the pure estimator (call update() per sample).

/// Body acceleration: linear x [m/s^2], angular z [rad/s^2].
struct BodyAccel
{
  double linear = 0.0;
  double angular = 0.0;
};

/// EMA-filtered acceleration estimator.
class AccelEstimator
{
public:
  /// alpha in (0,1]: weight of the new raw sample (upstream default 0.3).
  /// vel_epsilon: |velocity| below which we consider the axis stopped.
  explicit AccelEstimator(double alpha = 0.3, double vel_epsilon = 1e-4)
  : alpha_(alpha), vel_epsilon_(vel_epsilon) {}

  /// Feed a new velocity sample taken `dt` seconds after the previous one.
  /// Returns the current filtered acceleration estimate.
  BodyAccel update(const BodyVel & vel, double dt)
  {
    if (!primed_ || !std::isfinite(dt) || dt <= 0.0) {
      // First sample (or invalid dt): establish baseline, emit zero.
      prev_ = vel;
      primed_ = true;
      filtered_ = {};
      return filtered_;
    }

    const double raw_linear = (vel.linear - prev_.linear) / dt;
    const double raw_angular = (vel.angular - prev_.angular) / dt;

    filtered_.linear = filterAxis(filtered_.linear, raw_linear, vel.linear, prev_.linear);
    filtered_.angular = filterAxis(filtered_.angular, raw_angular, vel.angular, prev_.angular);

    prev_ = vel;
    return filtered_;
  }

  void reset()
  {
    primed_ = false;
    filtered_ = {};
    prev_ = {};
  }

private:
  double filterAxis(double prev_filtered, double raw, double v_now, double v_prev) const
  {
    // Snap to zero when both samples are effectively stopped (kills EMA tail).
    if (std::abs(v_now) < vel_epsilon_ && std::abs(v_prev) < vel_epsilon_) {
      return 0.0;
    }
    return alpha_ * raw + (1.0 - alpha_) * prev_filtered;
  }

  double alpha_;
  double vel_epsilon_;
  bool primed_ = false;
  BodyVel prev_;
  BodyAccel filtered_;
};

}  // namespace diff_drive

#endif  // DIFFERENTIAL_DRIVE_CONTROLLER_HPP_
