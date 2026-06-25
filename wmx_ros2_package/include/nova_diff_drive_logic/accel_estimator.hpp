// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
//
// Body-acceleration estimator with an exponential moving-average (EMA) filter,
// for the Nova `/odom_accel` topic. Ported from differential_drive_controller's
// send_odom_accel: raw derivative of body velocity, EMA-smoothed, snapped to
// zero when both current and previous velocity are ~0. WMX/ROS-free.
//
// NOTE: the upstream node also rate-limits publishing (publish every
// 1/accel_publish_rate). That cadence is a ROS-timer concern and is handled in
// the node layer; this class is the pure estimator (call update() per sample).
#ifndef NOVA_DIFF_DRIVE_LOGIC__ACCEL_ESTIMATOR_HPP_
#define NOVA_DIFF_DRIVE_LOGIC__ACCEL_ESTIMATOR_HPP_

#include <cmath>

#include "nova_diff_drive_logic/diff_drive_kinematics.hpp"

namespace nova_diff_drive_logic
{

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

}  // namespace nova_diff_drive_logic

#endif  // NOVA_DIFF_DRIVE_LOGIC__ACCEL_ESTIMATOR_HPP_
