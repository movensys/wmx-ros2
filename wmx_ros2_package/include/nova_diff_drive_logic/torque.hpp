// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License.
//
// Wheel torque feedback types + (placeholder) conversion. David asked for torque
// publishing; the only firm fact so far is that the per-axis value comes from
// WMX `CoreMotion::GetStatus().axesStatus[axis].actualTorque`
// (ref: baymax-wmx-server wmx_vehicle.cpp:101-112).
//
// TBD (confirm with Fikih / Movensys WMX team before relying on this):
//   - Units of actualTorque: Nm? % of rated torque? raw drive units? -> may need a scale factor.
//   - Is actualTorque guaranteed available on the Nova drives, or best-effort?
//   - Do we publish per-wheel torque, or also a derived chassis quantity?
//   - Sign convention vs wheel direction (flip_left/right) — align with kinematics.
#ifndef NOVA_DIFF_DRIVE_LOGIC__TORQUE_HPP_
#define NOVA_DIFF_DRIVE_LOGIC__TORQUE_HPP_

namespace nova_diff_drive_logic
{

/// Per-wheel torque feedback. Units TBD (see file header) — currently a
/// pass-through of the raw WMX actualTorque values.
struct WheelTorque
{
  double left = 0.0;
  double right = 0.0;
  bool available = false;  // TBD: set false when WMX can't provide torque this cycle.
};

/// Placeholder conversion from raw WMX actualTorque to the published value.
/// TBD: replace identity with the real unit conversion once units are confirmed.
inline WheelTorque convertRawTorque(double raw_left, double raw_right, bool available = true)
{
  // TBD: apply scale/offset here once Movensys confirms actualTorque units.
  return WheelTorque{raw_left, raw_right, available};
}

}  // namespace nova_diff_drive_logic

#endif  // NOVA_DIFF_DRIVE_LOGIC__TORQUE_HPP_
