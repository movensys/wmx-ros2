// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// Cartesian pose helpers ported from wmx_arm_server (SerdeCartesianPose /
// JsonCartesianOffset in comm_types.h of wmx-server-development). The WMX3
// CartesianPose stores translation in millimetres and orientation as ZYX Euler
// angles in degrees; ROS-facing vectors use metres and quaternions.

#ifndef ROBOT_POSE_UTILS_HPP_
#define ROBOT_POSE_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include "CoreMotionApi.h"
#include "RobotMotionApi.h"

namespace wmx_robot_option
{

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// Replace orig with repl only when repl is non-zero (mirrors replace_nonzero in
// constants.cpp, used for absolute jogging).
inline double replace_nonzero(double orig, double repl)
{
  return (repl != 0.0) ? repl : orig;
}

// Convert ZYX Euler angles (degrees) to a normalized quaternion.
inline void euler_to_quaternion(
  const wmx3Api::coordinate::EulerAngle & euler,
  double & qx, double & qy, double & qz, double & qw)
{
  double cy = std::cos(euler.w * DEG2RAD), sy = std::sin(euler.w * DEG2RAD);
  double cp = std::cos(euler.v * DEG2RAD), sp = std::sin(euler.v * DEG2RAD);
  double cr = std::cos(euler.u * DEG2RAD), sr = std::sin(euler.u * DEG2RAD);

  double r00 = cy * cp;
  double r01 = cy * sp * sr - sy * cr;
  double r02 = cy * sp * cr + sy * sr;
  double r10 = sy * cp;
  double r11 = sy * sp * sr + cy * cr;
  double r12 = sy * sp * cr - cy * sr;
  double r20 = -sp;
  double r21 = cp * sr;
  double r22 = cp * cr;

  double trace = r00 + r11 + r22;
  if (trace > 0.0) {
    double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (r21 - r12) * s;
    qy = (r02 - r20) * s;
    qz = (r10 - r01) * s;
  } else if (r00 > r11 && r00 > r22) {
    double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
    qw = (r21 - r12) / s;
    qx = 0.25 * s;
    qy = (r01 + r10) / s;
    qz = (r02 + r20) / s;
  } else if (r11 > r22) {
    double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
    qw = (r02 - r20) / s;
    qx = (r01 + r10) / s;
    qy = 0.25 * s;
    qz = (r12 + r21) / s;
  } else {
    double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
    qw = (r10 - r01) / s;
    qx = (r02 + r20) / s;
    qy = (r12 + r21) / s;
    qz = 0.25 * s;
  }

  double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;
}

// Convert a quaternion to ZYX Euler angles (degrees).
inline void quaternion_to_euler(
  double qx, double qy, double qz, double qw,
  wmx3Api::coordinate::EulerAngle & euler)
{
  double xx = qx * qx, yy = qy * qy, zz = qz * qz;
  double xy = qx * qy, xz = qx * qz, yz = qy * qz;
  double wx = qw * qx, wy = qw * qy, wz = qw * qz;

  double r00 = 1.0 - 2.0 * (yy + zz);
  double r01 = 2.0 * (xy - wz);
  double r02 = 2.0 * (xz + wy);
  double r10 = 2.0 * (xy + wz);
  double r11 = 1.0 - 2.0 * (xx + zz);
  double r12 = 2.0 * (yz - wx);
  double r20 = 2.0 * (xz - wy);
  r20 = std::clamp(r20, -1.0, 1.0);
  double r21 = 2.0 * (yz + wx);
  double r22 = 1.0 - 2.0 * (xx + yy);

  if (std::fabs(r20) >= 1.0) {
    euler.u = RAD2DEG * std::atan2(-r01, r11);
    euler.v = (r20 > 0.0) ? -90.0 : 90.0;
    euler.w = 0.0;
  } else {
    euler.u = RAD2DEG * std::atan2(r21, r22);
    euler.v = RAD2DEG * std::asin(-r20);
    euler.w = RAD2DEG * std::atan2(r10, r00);
  }

  while (euler.u < -180.0) {euler.u += 360.0;}
  while (euler.u >= 180.0) {euler.u -= 360.0;}
  while (euler.w < -180.0) {euler.w += 360.0;}
  while (euler.w >= 180.0) {euler.w -= 360.0;}
}

// WMX3 CartesianPose -> [x, y, z, qx, qy, qz, qw] (m, quaternion).
inline std::vector<double> pose_to_vector(const wmx3Api::coordinate::CartesianPose & pose)
{
  double qx, qy, qz, qw;
  euler_to_quaternion(pose.rotation, qx, qy, qz, qw);
  return {pose.point.x / 1000.0, pose.point.y / 1000.0, pose.point.z / 1000.0, qx, qy, qz, qw};
}

// [x, y, z, qx, qy, qz, qw] (m, quaternion) -> WMX3 CartesianPose (mm, deg).
inline wmx3Api::coordinate::CartesianPose vector_to_pose(const std::vector<double> & v)
{
  wmx3Api::coordinate::CartesianPose pose;
  pose.point.x = v[0] * 1000.0;
  pose.point.y = v[1] * 1000.0;
  pose.point.z = v[2] * 1000.0;
  quaternion_to_euler(v[3], v[4], v[5], v[6], pose.rotation);
  return pose;
}

// Add a 6-DoF offset [x, y, z, rx, ry, rz] (m, rad) onto an existing pose.
inline wmx3Api::coordinate::CartesianPose offset_pose(
  const wmx3Api::coordinate::CartesianPose & pose, const std::vector<double> & offset)
{
  return wmx3Api::coordinate::CartesianPose(
    pose.point.x + offset[0] * 1000.0,
    pose.point.y + offset[1] * 1000.0,
    pose.point.z + offset[2] * 1000.0,
    pose.rotation.u + offset[3] * RAD2DEG,
    pose.rotation.v + offset[4] * RAD2DEG,
    pose.rotation.w + offset[5] * RAD2DEG);
}

// Build an absolute pose, keeping the current value for any zero offset entry.
inline wmx3Api::coordinate::CartesianPose absolute_pose(
  const wmx3Api::coordinate::CartesianPose & pose, const std::vector<double> & offset)
{
  return wmx3Api::coordinate::CartesianPose(
    replace_nonzero(pose.point.x, offset[0] * 1000.0),
    replace_nonzero(pose.point.y, offset[1] * 1000.0),
    replace_nonzero(pose.point.z, offset[2] * 1000.0),
    replace_nonzero(pose.rotation.u, offset[3] * RAD2DEG),
    replace_nonzero(pose.rotation.v, offset[4] * RAD2DEG),
    replace_nonzero(pose.rotation.w, offset[5] * RAD2DEG));
}

}  // namespace wmx_robot_option

#endif  // ROBOT_POSE_UTILS_HPP_
